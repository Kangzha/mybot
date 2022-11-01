#! /usr/bin/env/python3
import sys

from pyparsing import line
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
#print(sys.path)
import rospy
import torch
import torch.nn as nn
import torch.optim as optim
import math
import random
import numpy as np
import time
from copy import deepcopy
from topic_example.msg import laserscan_filted
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
from collections import namedtuple
from collections import deque
from nav_msgs.msg import Odometry
import threading
from geometry_msgs.msg import Point,Quaternion
import tf
from tf.transformations import euler_from_quaternion

class ReplayMemory():
        
        def __init__(self, capacity):
            self.memory = deque(maxlen=capacity)

        def push(self, *args):
            self.memory.append(Experience(*args))
        
        def sample(self, batch_size):
            return random.sample(self.memory, batch_size)
        def __len__(self):
            return len(self.memory)


class ActorNet(nn.Module):
        def __init__(self, state_num, action_num, hidden1=500, hidden2=500, hidden3=500):
            super(ActorNet, self).__init__()
            self.fc1 = nn.Linear(state_num, hidden1)
            self.fc2 = nn.Linear(hidden1, hidden2)
            self.fc3 = nn.Linear(hidden2, hidden3)
            self.fc4 = nn.Linear(hidden3, 1)
            self.fc5 = nn.Linear(hidden3, 1)           
            self.relu = nn.ReLU()
            self.sigmoid = nn.Sigmoid()
            self.tanh = nn.Tanh()           

        def forward(self, x):
            x = self.relu(self.fc1(x))
            x = self.relu(self.fc2(x))
            x = self.relu(self.fc3(x))
            linear_x = self.sigmoid(self.fc4(x))
            angular_z = self.tanh(self.fc5(x))
            if linear_x.shape[0] == batch_size:
                out = torch.cat((linear_x,angular_z),dim=1)
            else:
                out = torch.cat((linear_x,angular_z),dim=0)
            
            return out

class CriticNet(nn.Module):
        def __init__(self, state_num, action_num, hidden1=500, hidden2=500, hidden3=500):
            super(CriticNet, self).__init__()
            self.fc1 = nn.Linear(state_num+action_num, hidden1)
            self.fc2 = nn.Linear(hidden1, hidden2)
            self.fc3 = nn.Linear(hidden2, hidden3)
            self.fc4 = nn.Linear(hidden3, 1)         
            self.relu = nn.ReLU()

        def forward(self, x):
            x = self.relu(self.fc1(x))
            x = self.relu(self.fc2(x))
            x = self.relu(self.fc3(x))
            out = self.fc4(x)
            return out
class robot_env:
        def __init__(self,
                     actor_net,
                     critic_net,
                     n_state = 28,
                     n_action = 2,
                     step_time = 0.1,
                     explore_noise = 0.2,
                     learn_rate = 0.0005,
                     gamma = 0.99,
                     update_weight = 0.999
                     ):
            self.n_state = n_state
            self.n_action = n_action
            self.robot_scan = torch.zeros(24).to(torch.float32)
            self.gamma = gamma
            self.update_weight = update_weight
            self.step_time = step_time
            self.learn_rate = learn_rate
            self.explore_noise = explore_noise
            

            self.position = Point()
            self.move_cmd = Twist()
            

            self.rate = rospy.Rate(100)
            self.target_x = -10
            self.target_y = -10
            
            self.linear_x =0.6
            self.angular_z = 0.2
            self.rate = rospy.Rate(100)
            
            self.pub_action = rospy.Publisher('/cmd_vel',Twist, queue_size=2)
            self.laser_info = rospy.Subscriber('/scan_filted',laserscan_filted,self.robot_scan_cb)
            
            self.tf_listener = tf.TransformListener()
            self.odom_frame = 'odom'
            self.tf_listener.waitForTransform(self.odom_frame,'base_footprint',rospy.Time(),rospy.Duration(1.0))
            self.base_frame = 'base_footprint'  
                
            (self.position,self.rotation) = self.get_odom()


            self.actor_net = actor_net
            self.critic_net = critic_net
            self.target_actor_net = deepcopy(actor_net)
            self.target_critic_net = deepcopy(critic_net)
            self.optimizer_actor = optim.Adam(actor_net.parameters(), lr=self.learn_rate)
            self.optimizer_critic = optim.Adam(critic_net.parameters(), lr=self.learn_rate)
            self.loss_fn = torch.nn.MSELoss()
            rospy.loginfo("Finish Subscriber Init...")

        def get_odom(self):
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.odom_frame,self.base_frame,rospy.Time(0))
                rotation = euler_from_quaternion(rot)
            except(tf.Exception,tf.ConnectivityException,tf.LookupException):
                rospy.loginfo("TF Exception")
                return
            return (Point(*trans),rotation[2])


        def print_odom(self):
            print("position x is %s, y is %s, rotation is %s",self.position.x, self.position.y, self.rotation)
            

        def explore_action(self, state):
            with torch.no_grad():
              action = self.actor_net(state)
              action = torch.normal(action, self.explore_noise)
            return action

        def sample_batch(self, experience_pool):
            experience = experience_pool.sample(batch_size)
            experience_batch = Experience(*zip(*experience))

            state_batch = torch.stack(experience_batch.state)
            action_batch = torch.stack(experience_batch.action)
            reward_batch = torch.stack(experience_batch.reward)
            next_state_batch = torch.stack(experience_batch.next_state)
            done_batch = torch.stack(experience_batch.done)
            state_action_batch = torch.cat((state_batch, action_batch), dim=1)
            return state_batch, action_batch, reward_batch, next_state_batch, done_batch, state_action_batch

        def robot_is_crashed(self, laser_values, range_limit):
            laser_crashed_reward = 0
            laser_crashed_value = 0
            for i in range(len(laser_values)):
                if(laser_values[i]<2*range_limit):
                    laser_crashed_reward = -80
                if(laser_values[i]<range_limit):
                    laser_crashed_reward = -200
                    laser_crashed_value = 1
                    
                    print("zhuang qiang")
                    
                    break
            laser_crashed_value = np.array(laser_crashed_value)
            laser_crashed_value = torch.from_numpy(laser_crashed_value)
            return laser_crashed_value,laser_crashed_reward

        
        def robot_scan_cb(self, data):
            #rospy.loginfo("laserscan_filted callback")
            for a in range(24):             
                self.robot_scan[a] = data.ranges[a]
		
        def enable_gradient(self, network):
            for p in network.parameters():
              p.requires_grad = True

        def disable_gradient(self, network):
            for p in network.parameters():
              p.requires_grad = False

        def copy_net(self, source_net, target_net):
            with torch.no_grad():
              for p, p_targ in zip(source_net.parameters(), target_net.parameters()):
                p_targ.data.mul_(self.update_weight)
                p_targ.data.add_((1 - self.update_weight) * p.data)

        def step(self,time_step=0.1, linear_x=0.8, angular_z=0.3):
            start_time = time.time()
            record_time_step = 0
            self.move_cmd.linear.x = linear_x      
            self.move_cmd.angular.z = angular_z
            self.rate.sleep()
            
            (self.position, self.rotation) = self.get_odom()
            self.print_odom()
            robot_x_previous = self.position.x
            robot_y_previous = self.position.y

            while (record_time_step < time_step):
                self.pub_action.publish(self.move_cmd)
                self.rate.sleep()
                record_time = time.time()
                record_time_step = record_time - start_time

            (self.position, self.rotation) = self.get_odom()
            self.print_odom()
            robot_x = self.position.x
            robot_y = self.position.y          
            current_distance_robot_target = math.sqrt((self.target_x - robot_x)**2 + (self.target_y - robot_y)**2)
            distance_robot_target_previous = math.sqrt((self.target_x - robot_x_previous)**2 + (self.target_y - robot_y_previous)**2)           
            rospy.loginfo("Previous_distance:%f, Current_distance:%f", distance_robot_target_previous, current_distance_robot_target)
            angle_robot = self.rotation
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
            rospy.loginfo("angle_diff:%f", angle_diff)
            normalized_laser = [(x)/3.5 for x in (self.robot_scan)]

            next_state = np.append(normalized_laser,current_distance_robot_target)
            next_state = np.append(next_state, angle_diff)
            next_state = np.append(next_state, linear_x)
            next_state = np.append(next_state, angular_z)          
            next_state = torch.tensor(next_state,dtype=torch.float)
            
            distance_reward = distance_robot_target_previous - current_distance_robot_target
            #distance_reward = -current_distance_robot_target
            rospy.loginfo("distance_reward:%f", distance_reward)

            laser_reward = sum(normalized_laser)-24
            #rospy.loginfo("laser_reward:%f", laser_reward)
            
            done ,laser_crash_reward = self.robot_is_crashed(self.robot_scan,0.25)
            #rospy.loginfo("laser_crash_reward:%f", laser_crash_reward)
            collision_reward = laser_crash_reward+laser_reward
            rospy.loginfo("laser_crash_reward:%f , laser_reward:%f", laser_crash_reward, laser_reward)

            angular_punish_reward = 0
            linear_punish_reward = 0
            angle_diff_reward = 0
            if angular_z >0.6 or angular_z < -0.6:
                angular_punish_reward = -1
                rospy.loginfo("angular_punish_reward:%f", angular_punish_reward)
            if linear_x < 0.2:
                linear_punish_reward = -2
                rospy.loginfo("linear_punish_reward:%f", linear_punish_reward)
            if angle_diff < 0.5 and angle_diff > -0.5:
                angle_diff_reward = 2
            arrive_reward = 0
            if current_distance_robot_target < 1:
                arrive_reward = 100
                rospy.loginfo("arrive_reward:%f", arrive_reward)
                done = 1
                done = np.array(done)
                done = torch.from_numpy(done)
            reward = distance_reward*(10/time_step)*8.4 + angle_diff_reward + arrive_reward + collision_reward + angular_punish_reward + linear_punish_reward
            #reward = distance_reward + arrive_reward + collision_reward + angular_punish_reward + linear_punish_reward

            
            return reward, next_state, done

        def update_actor_net(self, state):
            cur_action = self.actor_net(state)
            cur_sa = torch.cat((state, cur_action), dim=1)
            #cur_sa = torch.cat((state, cur_action), dim=0)
            self.disable_gradient(self.critic_net)
            a = self.critic_net(cur_sa)
            loss = -1.0 * torch.mean(a)
            #loss = -1.0 * a
            
            self.optimizer_actor.zero_grad()
            loss.backward()
            self.optimizer_actor.step()
            self.enable_gradient(self.critic_net)
            return loss.item()

        def update_critic_net(self, cur_reward, next_state, laser_crashed_value, sa):           
            cur_critic_value = np.squeeze(self.critic_net(sa)) 
            #cur_critic_value = self.critic_net(sa)          
            next_action = self.target_actor_net(next_state)
            next_sa = torch.cat((next_state, next_action), dim=1)
            #next_sa = torch.cat((next_state, next_action), dim=0)
            target_next_critic_value = np.squeeze(self.target_critic_net(next_sa))
            #target_next_critic_value = self.target_critic_net(next_sa)
            target_critic_value = cur_reward + self.gamma * target_next_critic_value * (1 - laser_crashed_value)
            
            loss = self.loss_fn(cur_critic_value, target_critic_value)           
            self.optimizer_critic.zero_grad()
            loss.backward()
            self.optimizer_critic.step()
            
            return loss.item()
        def reset(self):
            index_list = [-1, 1]
            index_x = np.random.choice(index_list)
            index_y = np.random.choice(index_list)

            # for maze
            #target_x = (np.random.random()-0.5)*5 + 12*index_x
            #target_y = (np.random.random()-0.5)*5 + 12*index_y
            #random_turtlebot_y = (np.random.random())*4 + index_turtlebot_y

            # for corridor
            self.target_x = (np.random.random()-0.5)*5 + 12*index_x
            self.target_y = (np.random.random()-0.5)*3
            random_turtlebot_y = (np.random.random())*5 #+ index_turtlebot_y

            state_msg = ModelState()    
            
            state_msg.model_name = 'mrobot'
            state_msg.pose.position.x = 0.0
            state_msg.pose.position.y = random_turtlebot_y
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.x = 0.0
            state_msg.pose.orientation.y = 0.0
            state_msg.pose.orientation.z = -0.2
            state_msg.pose.orientation.w = 0

            state_target_msg = ModelState()    
            state_target_msg.model_name = 'unit_sphere_0_0' 
            state_target_msg.pose.position.x = self.target_x
            state_target_msg.pose.position.y = self.target_y
            state_target_msg.pose.position.z = 0.0
            state_target_msg.pose.orientation.x = 0
            state_target_msg.pose.orientation.y = 0
            state_target_msg.pose.orientation.z = -0.2
            state_target_msg.pose.orientation.w = 0


            rospy.wait_for_service('gazebo/reset_simulation')
            try:
                rospy.ServiceProxy('gazebo/reset_simulation', Empty)
            except (rospy.ServiceException) as e:
                print("gazebo/reset_simulation service call failed")

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                set_state( state_msg )
                set_state( state_target_msg )
                print ("Gazebo_reset")
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


if __name__ == '__main__':
        #torch.autograd.set_detect_anomaly(True)
        train_epoch = 100
        step_epoch = 200
        batch_size = 128
        warm_up_size = batch_size
        rospy.init_node("train_ddpg", anonymous=True)
        actor_net = ActorNet(28, 2, hidden1=500, hidden2=500, hidden3=500)
        critic_net = CriticNet(28, 2, hidden1=500, hidden2=500, hidden3=500)
        env = robot_env(actor_net, critic_net)
        Experience = namedtuple('Experience', ('state', 'action', 'reward', 'next_state', 'done'))
        experience_pool = ReplayMemory(int(1e6))
               
        for e in range(train_epoch):
            state = env.reset()
            total_reward = 0
            
            for i in range(step_epoch):   
              #action = env.actor_net(state)
              action = env.explore_action(state)
              speed = action[0].item()
              angular = action[1].item()
              rospy.loginfo("speed:%f   angular:%f", speed,angular)
              reward, next_state, done = env.step(0.2, speed, angular)
              experience_pool.push(state, action, reward, next_state, done)

              state = next_state
              total_reward = total_reward + reward

              if len(experience_pool) > warm_up_size:
                  s, _, r, ns, d, sa = env.sample_batch(experience_pool)
                  loss_critic = env.update_critic_net(r, ns, d, sa)
                  loss_actor = env.update_actor_net(s)
                  if i % 10 == 0:
                      env.copy_net(env.actor_net, env.target_actor_net)
                      env.copy_net(env.critic_net, env.target_critic_net)                        
                  rospy.loginfo("train_epoch:%d   loss_actor:%f loss_critic:%f reward:%d", i,loss_actor,loss_critic,reward) 

              if done:
                  env.reset()
                  time.sleep(1)
                  break

            print("this is total_reward%d",total_reward)
 

'''
if __name__ == '__main__':
        #torch.autograd.set_detect_anomaly(True)
        train_epoch = 100
        step_epoch = 300
        batch_size = 128
        rospy.init_node("train_ddpg", anonymous=True)
        actor_net = ActorNet(28, 2, hidden1=500, hidden2=500, hidden3=500)
        critic_net = CriticNet(28, 2, hidden1=500, hidden2=500, hidden3=500)
        env = robot_env(actor_net, critic_net)
               
        for e in range(train_epoch):
            state = env.reset()
            total_reward = 0
            
            for i in range(step_epoch):   
              #action = env.actor_net(state)
              action = env.explore_action(state)
              speed = action[0].item()
              angular = action[1].item()
              rospy.loginfo("speed:%f  angular:%f", speed,angular)
              reward, next_state, done = env.step(0.2, speed, angular)
              total_reward = total_reward + reward
              sa = torch.cat((state, action), dim=0)
              loss_critic = env.update_critic_net(reward, next_state, done, sa)
              loss_actor = env.update_actor_net(state)
              state = next_state
              if i % 10 == 0:
                env.copy_net(env.actor_net, env.target_actor_net)
                env.copy_net(env.critic_net, env.target_critic_net)                        
              rospy.loginfo("train_epoch:%d   loss_actor:%f loss_critic:%f reward:%d", i,loss_actor,loss_critic,reward) 

              if done:
                  env.reset()
                  time.sleep(1)
                  

            print("this is total_reward%d",total_reward)
'''

