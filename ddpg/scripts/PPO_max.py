#! /usr/bin/env/python3
from os import stat
import sys

from pyparsing import line
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
#print(sys.path)
import argparse
import rospy
import torch
import torch.nn as nn
import torch.optim as optim
import math
import random
import numpy as np
import time
from topic_example.msg import laserscan_filted
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
from collections import namedtuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Quaternion
import tf
from tf.transformations import euler_from_quaternion
from torch.distributions import Normal
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler

class ReplayBuffer:
    def __init__(self, args):
        self.s = np.zeros((args.batch_size, args.state_dim))
        self.a = np.zeros((args.batch_size, args.action_dim))
        self.a_logprob = np.zeros((args.batch_size, args.action_dim))
        self.r = np.zeros((args.batch_size, 1))
        self.s_ = np.zeros((args.batch_size, args.state_dim))
        self.dw = np.zeros((args.batch_size, 1))
        self.done = np.zeros((args.batch_size, 1))
        self.count = 0

    def store(self, s, a, a_logprob, r, s_, dw, done):
        self.s[self.count] = s
        self.a[self.count] = a
        self.a_logprob[self.count] = a_logprob
        self.r[self.count] = r
        self.s_[self.count] = s_
        self.dw[self.count] = dw
        self.done[self.count] = done
        self.count += 1

    def numpy_to_tensor(self):
        s = torch.tensor(self.s, dtype=torch.float)
        a = torch.tensor(self.a, dtype=torch.float)
        a_logprob = torch.tensor(self.a_logprob, dtype=torch.float)
        r = torch.tensor(self.r, dtype=torch.float)
        s_ = torch.tensor(self.s_, dtype=torch.float)
        dw = torch.tensor(self.dw, dtype=torch.float)
        done = torch.tensor(self.done, dtype=torch.float)

        return s, a, a_logprob, r, s_, dw, done

class Actor_Gaussian(nn.Module):
    def __init__(self, args):
        super(Actor_Gaussian, self).__init__()
        self.fc1 = nn.Linear(args.state_dim, args.hidden_width)
        self.fc2 = nn.Linear(args.hidden_width, args.hidden_width)
        self.mean_layer = nn.Linear(args.hidden_width, args.action_dim)
        self.log_std = nn.Parameter(torch.zeros(1, args.action_dim))  # We use 'nn.Parameter' to train log_std automatically
        self.activate_func1 = nn.ReLU() 
        self.activate_func2 = nn.Tanh()


    def forward(self, s):
        s = self.activate_func1(self.fc1(s))
        s = self.activate_func1(self.fc2(s))
        mean = self.activate_func2(self.mean_layer(s))*0.3  # [-1,1]->[-max_action,max_action]
        return mean

    def get_dist(self, s):
        mean = self.forward(s)

        log_std = self.log_std.expand_as(mean)  # To make 'log_std' have the same dimension as 'mean'
        std = torch.exp(log_std)  # The reason we train the 'log_std' is to ensure std=exp(log_std)>0
        dist = Normal(mean, std)  # Get the Gaussian distribution
        return dist


class Critic(nn.Module):
    def __init__(self, args):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(args.state_dim, args.hidden_width)
        self.fc2 = nn.Linear(args.hidden_width, args.hidden_width)
        self.fc3 = nn.Linear(args.hidden_width, 1)
        self.activate_func = nn.ReLU()


    def forward(self, s):
        s = self.activate_func(self.fc1(s))
        s = self.activate_func(self.fc2(s))
        v_s = self.fc3(s)
        return v_s


class PPO_continuous():
    def __init__(self, args):
        self.policy_dist = args.policy_dist
        #self.max_action = args.max_action
        self.batch_size = args.batch_size
        self.mini_batch_size = args.mini_batch_size
        self.max_train_steps = args.max_train_steps
        self.lr_a = args.lr_a  # Learning rate of actor
        self.lr_c = args.lr_c  # Learning rate of critic
        self.gamma = args.gamma  # Discount factor
        self.lamda = args.lamda  # GAE parameter
        self.epsilon = args.epsilon  # PPO clip parameter
        self.K_epochs = args.K_epochs  # PPO parameter
        self.entropy_coef = args.entropy_coef  # Entropy coefficient
        #self.set_adam_eps = args.set_adam_eps
        self.use_grad_clip = args.use_grad_clip
        self.use_lr_decay = args.use_lr_decay
        self.use_adv_norm = args.use_adv_norm

        self.actor = Actor_Gaussian(args)
        self.critic = Critic(args)

        self.optimizer_actor = torch.optim.Adam(self.actor.parameters(), lr=self.lr_a)
        self.optimizer_critic = torch.optim.Adam(self.critic.parameters(), lr=self.lr_c)

        self.loss_fn = torch.nn.MSELoss()

    def evaluate(self, s):  # When evaluating the policy, we only use the mean
        s = torch.unsqueeze(torch.tensor(s, dtype=torch.float), 0)       
        a = self.actor(s).detach().numpy().flatten()
        return a

    def choose_action(self, s):
        s = torch.unsqueeze(torch.tensor(s, dtype=torch.float), 0)
        with torch.no_grad():
            dist = self.actor.get_dist(s)
            a = dist.sample()  # Sample the action according to the probability distribution
            a[0][0] = torch.clamp(a[0][0], -0.3, 0.3)  # [-max,max]
            a[0][1] = torch.clamp(a[0][1], -0.6, 0.6)
            #a = torch.clamp(a, -0.3, 0.3)
            a_logprob = dist.log_prob(a)  # The log probability density of the action
        return a.numpy().flatten(), a_logprob.numpy().flatten()

    def update(self, replay_buffer):
        s, a, a_logprob, r, s_, dw, done = replay_buffer.numpy_to_tensor()  # Get training data
        adv = []
        gae = 0
        with torch.no_grad():  # adv and v_target have no gradient
            vs = self.critic(s)
            vs_ = self.critic(s_)
            deltas = r + self.gamma * (1.0 - dw) * vs_ - vs  
            for delta, d in zip(reversed(deltas.flatten().numpy()), reversed(done.flatten().numpy())):
                gae = delta + self.gamma * self.lamda * gae * (1.0 - d) 
                adv.insert(0, gae)
            adv = torch.tensor(adv, dtype=torch.float).view(-1, 1) #状态的真实得分和评论家打分的差距
            v_target = adv + vs    #critic网络的输出vs需要逼近的目标值
        # Optimize policy for K epochs:
        for _ in range(self.K_epochs):
            # Random sampling and no repetition. 'False' indicates that training will continue even if the number of samples in the last time is less than mini_batch_size
            for index in BatchSampler(SubsetRandomSampler(range(self.batch_size)), self.mini_batch_size, False):
                dist_now = self.actor.get_dist(s[index])
                a_logprob_now = dist_now.log_prob(a[index])
                # a/b=exp(log(a)-log(b))  In multi-dimensional continuous action space，we need to sum up the log_prob
                ratios = torch.exp(a_logprob_now.sum(1, keepdim=True) - a_logprob[index].sum(1, keepdim=True))  # shape(mini_batch_size X 1)

                surr1 = ratios * adv[index]  # Only calculate the gradient of 'a_logprob_now' in ratios
                surr2 = torch.clamp(ratios, 1 - self.epsilon, 1 + self.epsilon) * adv[index]
                actor_loss = -torch.min(surr1, surr2) 
                # Update actor
                self.optimizer_actor.zero_grad()
                actor_loss.mean().backward()
                self.optimizer_actor.step()

                v_s = self.critic(s[index])
                critic_loss = self.loss_fn(v_target[index], v_s).requires_grad_(True)
                # Update critic
                self.optimizer_critic.zero_grad()
                critic_loss.backward()
                self.optimizer_critic.step()

class robot_env:
    def __init__(self,):
        self.n_state = 28
        self.robot_scan = torch.zeros(24).to(torch.float32)        
        self.position = Point()
        self.move_cmd = Twist()
        self.rate = rospy.Rate(100)
        self.target_x = 10.0
        self.target_y = -10
        self.linear_x =0.6
        self.angular_z = 0.2
        self.pub_action = rospy.Publisher('/cmd_vel',Twist, queue_size=2)
        self.laser_info = rospy.Subscriber('/scan_filted',laserscan_filted,self.robot_scan_cb)
        
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.tf_listener.waitForTransform(self.odom_frame,'base_footprint',rospy.Time(),rospy.Duration(1.0))
        self.base_frame = 'base_footprint'  
            
        (self.position,self.rotation) = self.get_odom()

    def get_odom(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.odom_frame,self.base_frame,rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except(tf.Exception,tf.ConnectivityException,tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans),rotation[2])


    #def print_odom(self):
        #print("x方向位置: , y方向位置: ",self.position.x, self.position.y, self.rotation)


    def robot_is_crashed(self, laser_values, range_limit):
        laser_crashed_reward = 0
        laser_crashed_value = 0
        for i in range(len(laser_values)):
            if(laser_values[i]<2*range_limit):
                laser_crashed_reward = -100
            if(laser_values[i]<range_limit):
                laser_crashed_reward = -250
                laser_crashed_value = 1        
                print("撞墙了")
                self.reset()
                break
        return laser_crashed_value,laser_crashed_reward

    
    def robot_scan_cb(self, data):
        #rospy.loginfo("laserscan_filted callback")
        for a in range(24):             
            self.robot_scan[a] = data.ranges[a]


    def step(self,time_step=0.2, linear_x=0.8, angular_z=0.3):
        start_time = time.time()
        record_time_step = 0
        self.move_cmd.linear.x = linear_x      
        self.move_cmd.angular.z = angular_z
        self.rate.sleep()
        
        (self.position, self.rotation) = self.get_odom()
        robot_x_previous = self.position.x
        robot_y_previous = self.position.y

        while (record_time_step < time_step):
            self.pub_action.publish(self.move_cmd)
            self.rate.sleep()
            record_time = time.time()
            record_time_step = record_time - start_time

        (self.position, self.rotation) = self.get_odom()
        
        robot_x = self.position.x
        robot_y = self.position.y          
        current_distance_robot_target = math.sqrt((self.target_x - robot_x)**2 + (self.target_y - robot_y)**2)
        distance_robot_target_previous = math.sqrt((self.target_x - robot_x_previous)**2 + (self.target_y - robot_y_previous)**2)           
        #rospy.loginfo("Previous_distance:%f, Current_distance:%f", distance_robot_target_previous, current_distance_robot_target)
        angle_robot = self.rotation
        #print("yaw:",angle_robot)
        angle_robot_target = math.atan2(self.target_y-robot_y,self.target_x - robot_x)

        if angle_robot <0:
            angle_robot = angle_robot+2*math.pi
        if angle_robot_target <0:
            angle_robot_target = angle_robot_target + 2*math.pi

        angle_diff = angle_robot_target - angle_robot
        if angle_diff < -math.pi:
            angle_diff = angle_diff + 2*math.pi
        if angle_diff > math.pi:
            angle_diff = angle_diff - 2*math.pi
        #rospy.loginfo("angle_diff:%f", angle_diff)
        normalized_laser = [(x)/8.0 for x in (self.robot_scan)]

        next_state = np.append(normalized_laser,current_distance_robot_target)
        next_state = np.append(next_state, angle_diff)
        next_state = np.append(next_state, linear_x)
        next_state = np.append(next_state, angular_z   )   
        next_state = torch.tensor(next_state,dtype=torch.float)
        
        distance_reward = (distance_robot_target_previous - current_distance_robot_target)*1650
        
        #rospy.loginfo("distance_reward:%f", distance_reward)

        laser_reward = (sum(normalized_laser)-24)
        #laser_reward = 0
        #rospy.loginfo("laser_reward:", laser_reward)
        
        done ,laser_crash_reward = self.robot_is_crashed(self.robot_scan,0.25)
        #rospy.loginfo("laser_crash_reward:%f", laser_crash_reward)
        collision_reward = laser_crash_reward+laser_reward
        #rospy.loginfo("collision reward:%f", collision_reward)

        angular_punish_reward = 0
        linear_punish_reward = 0
        angle_diff_reward = 0
        #if angular_z >0.6 or angular_z < -0.6:
            #angular_punish_reward = -2
            #rospy.loginfo("angular_punish_reward:%f", angular_punish_reward)
        #if linear_x > 0.3 or linear_x <-0.3:
            #linear_punish_reward = -2
            #rospy.loginfo("linear_punish_reward:%f", linear_punish_reward)
        if angle_diff > 0.5 or angle_diff < -0.5:
            angle_diff_reward = -2
        #rospy.loginfo("angle_reward:%f", angle_diff_reward)
        arrive_reward = 0
        if current_distance_robot_target < 0.5:
            arrive_reward = 200
            rospy.loginfo("抵达目标地点")
            done = 1
            self.reset()

        reward = distance_reward + angle_diff_reward + arrive_reward + \
        collision_reward + angular_punish_reward + linear_punish_reward

        return reward, next_state, done


    def reset(self):
        position_x_list = [10]
        position_y_list = [-10]
        x = np.random.choice(position_x_list)
        y = np.random.choice(position_y_list)


        self.target_x = x
        self.target_y = y
        
        state_msg = ModelState()    
        
        state_msg.model_name = 'mrobot'
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = 0.5
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.2

        state_msg.pose.orientation.w = 1.55

        state_target1_msg = ModelState()    
        state_target1_msg.model_name = 'C1' 
        state_target1_msg.pose.position.x = 1
        state_target1_msg.pose.position.y = np.random.choice([-4,-5,-6,-7,-8])
        state_target1_msg.pose.position.z = 1.0
        state_target1_msg.pose.orientation.x = 0
        state_target1_msg.pose.orientation.y = 0
        state_target1_msg.pose.orientation.z = -0.2
        state_target1_msg.pose.orientation.w = 0

        state_target2_msg = ModelState()    
        state_target2_msg.model_name = 'C2' 
        state_target2_msg.pose.position.x = np.random.randint(10,20,size=1)
        state_target2_msg.pose.position.y = np.random.choice([-0.8,0.8])
        state_target2_msg.pose.position.z = 1.0
        state_target2_msg.pose.orientation.x = 0
        state_target2_msg.pose.orientation.y = 0
        state_target2_msg.pose.orientation.z = -0.2
        state_target2_msg.pose.orientation.w = 0


        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state( state_msg )
            set_state( state_target1_msg )
            set_state( state_target2_msg )
            print ("Gazebo_reset")
            print ("target pos:",self.target_x,self.target_y)
        except rospy.ServiceException:
            print ("Service call failed: %s")

        initial_state = np.ones(self.n_state)
        initial_state[self.n_state-1] = 0
        initial_state[self.n_state-2] = 0
        initial_state[self.n_state-3] = 0
        initial_state[self.n_state-4] = 0

        self.linear_x=0
        self.angular_z=0
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.pub_action.publish(self.move_cmd)
        time.sleep(1)
        initial_state = torch.tensor(initial_state,dtype=torch.float)
        return initial_state

def evaluate_policy(env, agent):
    evaluate_reward = 0
    s = env.reset()
    done = False
    steps = 0
    while not done  :
        a = agent.evaluate(s)  
        #print("a[0]=%f,a[1]=%f",abs(a[0]),a[1])
        r, s_, done = env.step(0.2,abs(a[0]),a[1])
        evaluate_reward += r
        s = s_
        steps += 1
    return evaluate_reward

def main(args):
    args.state_dim = 28
    args.action_dim = 2
    args.max_episode_steps = 200
    env = robot_env()
    total_steps = 0 
    replay_buffer = ReplayBuffer(args)
    agent = PPO_continuous(args)
    evaluate_steps = 1
    s = env.reset()
    agent.actor.load_state_dict(torch.load("actor_net.pth"))
    agent.critic.load_state_dict(torch.load("critic_net.pth"))
    while total_steps < args.max_train_steps: 
        episode_steps = 0
        done = False
        while not done:
            episode_steps += 1
            a, a_logprob = agent.choose_action(s)
            r,s_, done= env.step(0.2,abs(a[0]),a[1])
            if done and episode_steps != args.max_episode_steps:
                dw = True
            else:
                dw = False
            replay_buffer.store(s, a, a_logprob, r, s_, dw, done)
            s = s_
            
            if replay_buffer.count == args.batch_size:
                agent.update(replay_buffer)
                rospy.loginfo("update")
                replay_buffer.count = 0
                evaluate_steps += 1
            
            if evaluate_steps % 10 == 0:
                rospy.loginfo("*************************evaluate************************")
                evaluate_reward = evaluate_policy(env, agent)
                print("evaluate_reward:%f",evaluate_reward)
                evaluate_steps = 1
        total_steps += 1
    torch.save(agent.actor.state_dict(), "actor_net.pth")
    torch.save(agent.critic.state_dict(), "critic_net.pth")

if __name__ == '__main__':
    rospy.init_node("train_ddpg", anonymous=True)
    parser = argparse.ArgumentParser("Hyperparameters Setting for PPO-continuous")
    parser.add_argument("--max_train_steps", type=int, default=200, help=" Maximum number of training steps")
    parser.add_argument("--evaluate_freq", type=float, default=5e3, help="Evaluate the policy every 'evaluate_freq' steps")
    parser.add_argument("--save_freq", type=int, default=50, help="Save frequency")
    parser.add_argument("--policy_dist", type=str, default="Gaussian", help="Beta or Gaussian")
    #parser.add_argument("--batch_size", type=int, default=200, help="Batch size")   #for maze
    parser.add_argument("--batch_size", type=int, default=200, help="Batch size")
    #parser.add_argument("--mini_batch_size", type=int, default=40, help="Minibatch size")  #for maze
    parser.add_argument("--mini_batch_size", type=int, default=50, help="Minibatch size")
    parser.add_argument("--hidden_width", type=int, default=500, help="The number of neurons in hidden layers of the neural network")
    parser.add_argument("--lr_a", type=float, default=0.0005, help="Learning rate of actor")
    parser.add_argument("--lr_c", type=float, default=0.0005, help="Learning rate of critic")
    parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
    parser.add_argument("--lamda", type=float, default=0.85, help="GAE parameter")
    parser.add_argument("--epsilon", type=float, default=0.2, help="PPO clip parameter")
    parser.add_argument("--K_epochs", type=int, default=5, help="PPO parameter")
    parser.add_argument("--use_adv_norm", type=bool, default=True, help="Trick 1:advantage normalization")
    parser.add_argument("--use_state_norm", type=bool, default=True, help="Trick 2:state normalization")
    parser.add_argument("--use_reward_norm", type=bool, default=False, help="Trick 3:reward normalization")
    parser.add_argument("--use_reward_scaling", type=bool, default=True, help="Trick 4:reward scaling")
    parser.add_argument("--entropy_coef", type=float, default=0.01, help="Trick 5: policy entropy")
    parser.add_argument("--use_lr_decay", type=bool, default=True, help="Trick 6:learning rate Decay")
    parser.add_argument("--use_grad_clip", type=bool, default=True, help="Trick evaluate_steps = 1")
    args = parser.parse_args()
    main(args)
