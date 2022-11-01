#include "mbot_linux_serial.h"

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::system::error_code err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

//发送左右轮速控制速度共用体
union sendData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;
union ReceiveData
{
	int d;
	unsigned char data[4];
}Yaw1,Roll1,Pitch1;
//接收数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
// union receiveData
// {
// 	short d;
// 	unsigned char data[2];
// }leftVelNow,rightVelNow,angleNow,Yaw0,Roll0,Pitch0,X,Y,Z;
union receiveData
{
	short d;
	unsigned char data[2];
}Yaw0,Roll0,Pitch0,Accx,Accy,Accz,Gyrox,Gyroy,Gyroz,Magx,Magy,Magz,sFlag;

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

/********************************************************
函数功能：将对机器人的左右轮子控制速度，打包发送给下位机
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
void writeSpeed(double Left_v, double Right_v,unsigned char ctrlFlag)
{
    unsigned char buf[11] = {0};//
    int i, length = 0;

    leftVelSet.d  = Left_v;//mm/s
    rightVelSet.d = Right_v;

    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置机器人左右轮速度
    length = 5;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = leftVelSet.data[i];  //buf[3] buf[4]
        buf[i + 5] = rightVelSet.data[i]; //buf[5] buf[6]
    }
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[7]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[8]
    buf[3 + length + 1] = ender[0];     //buf[9]
    buf[3 + length + 2] = ender[1];     //buf[10]

    // 通过串口下发数据
    boost::asio::write(sp, boost::asio::buffer(buf));
}
/********************************************************
函数功能：从下位机读取数据
入口参数：机器人左轮轮速、右轮轮速、角度，预留控制位
出口参数：bool
********************************************************/
// bool readSpeed(double &Left_v,double &Right_v,double &yaw,double &roll,double &pitch,double &x,double &y,double &z,unsigned char &ctrlFlag)
// {
//     char i, length = 0;
//     unsigned char checkSum;
//     unsigned char buf[500]={0};
//     //=========================================================
//     //此段代码可以读数据的结尾，进而来进行读取数据的头部
//     try

//     {
//         boost::asio::streambuf response;
//         boost::asio::read_until(sp, response, "\r\n",err);   
//         copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
//         istream_iterator<unsigned char>(),
//         buf); 
//     }  
//     catch(boost::system::system_error &err)
//     {
//         ROS_INFO("read_until error");
//     } 
//     //=========================================================        

//     // 检查信息头
//     if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
//     {
//         ROS_ERROR("Received message header error!");
//         return false;
//     }
//     // 数据长度
//     length = buf[2];                                 //buf[2]

//     // 检查信息校验值
//     checkSum = getCrc8(buf, 3 + length);             //buf[10] 计算得出
//     if (checkSum != buf[3 + length])                 //buf[10] 串口接收
//     {
//         ROS_ERROR("Received data check sum error!");
//         return false;
//     }    

//     // 读取速度值
//     for(i = 0; i < 2; i++)
//     {
//         leftVelNow.data[i]  = buf[i + 3]; //buf[3] buf[4]
//         rightVelNow.data[i] = buf[i + 5]; //buf[5] buf[6]
//         //angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
//         Yaw0.data[i]        = buf[i+7];   //buf[7] buf[8]
//         Roll0.data[i]       = buf[i+9];   //buf[9] buf[10]
//         Pitch0.data[i]       = buf[i+11];   //buf[11] buf[12]
//         X.data[i]       = buf[i+13];   //buf[13] buf[114]
//         Y.data[i]       = buf[i+15];   //buf[15] buf[16]
//         Z.data[i]       = buf[i+17];   //buf[17] buf[18]
//     }

//     // 读取控制标志位
//     //ctrlFlag = buf[9];
//     ctrlFlag = buf[19];
//     Left_v  =leftVelNow.d;
//     Right_v =rightVelNow.d;
//     //Angle   =angleNow.d;
//     yaw     =Yaw0.d;
//     roll    =Roll0.d;
//     pitch   =Pitch0.d;
//     x       =X.d;
//     y       =Y.d;
//     z       =Z.d;
//     return true;
// }
bool readSpeed(float &yaw,float &roll,float &pitch,float &accx,float &accy,float &accz,float &gyrox,float &gyroy,float &gyroz,float &mx,float &my,float &mz,short &flag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[3000]={0};
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    try

    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n",err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        buf); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        

    // 检查信息头
    if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 数据长度
    length = buf[2];                                 //buf[2]

    // 检查信息校验值
    checkSum = getCrc8(buf, 3 + length);             //buf[10] 计算得出
    if (checkSum != buf[3 + length])                 //buf[10] 串口接收
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }    
    for(i=0;i<4;i++)
    {
	Yaw1.data[i] = buf[i+3];
        Roll1.data[i] = buf[i+7];
        Pitch1.data[i] = buf[i+11];
    }
    // 读取速度值
    for(i = 0; i < 2; i++)
    {
        Accx.data[i]       = buf[i+15];   //buf[9] buf[10]
        Accy.data[i]       = buf[i+17];   //buf[11] buf[12]
        Accz.data[i]       = buf[i+19];   //buf[13] buf[14]
        Gyrox.data[i]       = buf[i+21];   //buf[9] buf[10]
        Gyroy.data[i]       = buf[i+23];   //buf[11] buf[12]
        Gyroz.data[i]       = buf[i+25];   //buf[13] buf[14]
        Magx.data[i]       = buf[i+27];   //buf[9] buf[10]
        Magy.data[i]       = buf[i+29];   //buf[11] buf[12]
        Magz.data[i]       = buf[i+31];   //buf[13] buf[14]
        sFlag.data[i]      =buf[i+33];
    }

    yaw     =Yaw1.d;
    roll    =Roll1.d;
    pitch   =Pitch1.d;
    accx       =Accx.d;
    accy       =Accy.d;
    accz       =Accz.d;
    gyrox   =Gyrox.d;
    gyroy   =Gyroy.d;
    gyroz   =Gyroz.d;
    mx      =Magx.d;
    my      =Magy.d;
    mz      =Magz.d;
    flag    =sFlag.d;
    return true;
}
/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
