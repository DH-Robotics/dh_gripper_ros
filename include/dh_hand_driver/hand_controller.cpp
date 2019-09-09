/**
This is the control interface for the DH Hand
Author:   Jie Sun
Email:    jie.sun@dh-robotics.com
Date:     2019 July 1

Version 2.0
Copyright @ DH-Robotics Ltd.  
**/

#include <dh_hand_driver/hand_controller.h>
HandController::HandController(ros::NodeHandle n,const std::string &name)
: as_(n, name, boost::bind(&HandController::actuateHandCB, this, _1), false)
{
  n.param<std::string>("Connect_port", hand_port_name_, "/dev/DH_hand"); 
  n.param<std::string>("Hand_Model",  Hand_Model_    , "AG-2E"); 
  n.param<double>("WaitDataTime",  WaitDataTime_    , 0.5); 
  ROS_INFO("Hand_model : %s",Hand_Model_.c_str());
  ROS_INFO("Connect_port: %s",hand_port_name_.c_str());

  connect_mode = 0;
  if(hand_port_name_.find('/')!=std::string::npos)
  {
    connect_mode = 1;
  }
  if(hand_port_name_.find(':')!=std::string::npos)
  {
    connect_mode = 2;
  }
  ROS_INFO("connect_mode : %d",connect_mode);
   if(!build_conn())
   {
     return;
   }
  initHand();

  ros::ServiceServer service = n.advertiseService("hand_joint_state", &HandController::jointValueCB, this);
  as_.start();
  ros::spin();

}

HandController::~HandController()
{
  closeDevice();
}

bool HandController::jointValueCB(dh_hand_driver::hand_state::Request  &req,
         dh_hand_driver::hand_state::Response &res)
{
  W_mutex.lock();
  readtempdata.DataStream_clear();
  switch(req.get_target)
  {
    case 0: ROS_INFO("request: getMotorForce");     Hand.getMotorForce();     break;
    case 1: ROS_INFO("request: getMotor1Position"); Hand.getMotorPosition(1); break;
    case 2: 
      ROS_INFO("request: getMotor2Position"); 
      if(Hand_Model_=="AG-3E")
      {
        Hand.getMotorPosition(2);
        break;
      }
    default: ROS_ERROR("invalid read command");  W_mutex.unlock();return false; break;
  }
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
    Readdata();
  R_mutex.unlock();
  res.return_data = readtempdata.data[0];
  return true;
}



void HandController::actuateHandCB(const dh_hand_driver::ActuateHandGoalConstPtr &goal)
{
    ROS_INFO("Start to move the DH %s Hand" , Hand_Model_.c_str() );

    dh_hand_driver::ActuateHandFeedback feedback;
    dh_hand_driver::ActuateHandResult result;
    bool succeeded = false;
    bool bFeedback = false;

    // move Motor
    if(Hand_Model_=="AG-2E"&&goal->MotorID==2)
    {
      ROS_ERROR("invalid AG-2E command");
      as_.setAborted(result);
      return;
    }

    if(goal->force >= 0 && goal->position >=0 && goal->position <= 100)
    { 
      setGrippingForce(goal->force);
      succeeded = moveHand(goal->MotorID,goal->position,bFeedback);
      feedback.position_reached = bFeedback;
      as_.publishFeedback(feedback);
      ros::Duration(WaitDataTime_).sleep();
      result.opration_done = succeeded;
    }
    else
    {
     ROS_ERROR("received invalid action command");
    }
    if(succeeded)
      as_.setSucceeded(result);
    else
      as_.setAborted(result);

}

/**
 * To build the communication with the servo motors
 *
* */
bool HandController::build_conn()
{

  bool hand_connected = false;
  if(connect_mode==1)
  {
    try
    {
        hand_ser_.setPort(hand_port_name_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        hand_ser_.setTimeout(to);
        hand_ser_.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port of the hand");
        return hand_connected;
    }

    if(hand_ser_.isOpen())
    {
        ROS_INFO_STREAM("Serial Port for hand initialized");
        hand_connected = true;
    }
    else
    {
        return hand_connected;
    }
  }
  else if(connect_mode==2)
  {
    
    std::string servInetAddr = hand_port_name_.substr(0,hand_port_name_.find(":"));
    int PORT = atoi(hand_port_name_.substr(hand_port_name_.find(":")+1,hand_port_name_.size()-hand_port_name_.find(":")-1).c_str());
  
    /*创建socket*/ 
       struct sockaddr_in serv_addr; 
    if ((sockfd = socket(AF_INET,SOCK_STREAM,0)) != -1) 
    { 
      ROS_INFO("Socket id = %d",sockfd); 
      /*设置sockaddr_in 结构体中相关参数*/ 
      serv_addr.sin_family = AF_INET; 
      serv_addr.sin_port = htons(PORT); 
      inet_pton(AF_INET, servInetAddr.c_str(), &serv_addr.sin_addr);  
      bzero(&(serv_addr.sin_zero), 8); 
      /*调用connect 函数主动发起对服务器端的连接*/ 
      if(connect(sockfd,(struct sockaddr *)&serv_addr, sizeof(serv_addr))== -1) 
      { 
        perror("Connect failed!\n"); 
        hand_connected = false;
      } 
      else
      {
        ROS_INFO("connected"); 
        hand_connected = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Socket failed!\n"); 
      hand_connected = false;
    }
  }
  return hand_connected;
}



void HandController::closeDevice()
{
  // stop communication
  if(connect_mode==1)
    hand_ser_.close();
  else if(connect_mode==2)
     close(sockfd);
}

void HandController::initHand()
{
   // send initialize commond 
  W_mutex.lock();
    Hand.setInitialize();
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
     Readdata();
  R_mutex.unlock();

    ros::Duration(5).sleep();

  R_mutex.lock();
     Readdata();
  R_mutex.unlock();
}

bool HandController::moveHand(int MotorID, int target_position,bool &feedback)
{
  W_mutex.lock();
  Hand.setMotorPosition(MotorID,target_position);
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
    Readdata();
  R_mutex.unlock();

  ros::Duration(WaitDataTime_).sleep();

  W_mutex.lock();
    Hand.getFeedback(MotorID);
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
  if(Readdata())
  {
      R_mutex.unlock();
    if(readtempdata.data[0]==2)
        feedback = true;
    else
        feedback = false;

    return true;
  }
  else
  {
    R_mutex.unlock();
    return false;
  }
}

void HandController::setGrippingForce(int gripping_force)
{
  W_mutex.lock();
    Hand.setMotorForce(gripping_force);
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
    Readdata();
  R_mutex.unlock();

}

bool HandController::Writedata(std::vector<uint8_t> data)
{
  // ROS_INFO("send x");
  if(connect_mode==1)
  {
    if(hand_ser_.write(data)==data.size())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if(connect_mode==2)
  {
    if(write(sockfd,data.data(),data.size())==(unsigned int)data.size())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  return true;
}

// can not process Special case
bool HandController::Readdata()
{ 
  static int HadOvertime = 0;
  
  int i = WaitDataTime_ / 0.05;

  if(connect_mode == 1)
  {
    while(i--)
    {
      ros::Duration(0.05).sleep();
      if(hand_ser_.available()<14&&i==1)
      {
        ROS_ERROR_STREAM("Read Overtime you can increase 'WaitDataTime' in launch file ");
        HadOvertime++;
        return false;
      }
      else if(hand_ser_.available()>=14)
      {
          uint8_t buf[14];
          if(HadOvertime&&hand_ser_.available()>14)
          {
            ROS_INFO("Read last");
            for(;HadOvertime>0;HadOvertime--)
              hand_ser_.read(buf,14);
          }
          hand_ser_.read(buf,14);
          readtempdata.DatafromStream(buf,14);
          ROS_INFO("Read: %X %X %X %X %X %X %X %X %X %X %X %X %X %X",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13]);
          return true;
        }
    }
  }
  else if(connect_mode==2)
  {
    std::vector<uint8_t> temp;
    unsigned char tempbuf[100]={0};
    int get_num = 0;
    while(i--)
    {
      ros::Duration(0.05).sleep();
      get_num = read(sockfd,tempbuf,100);
      for(int i = 0;i<get_num;i++)
        temp.push_back(tempbuf[i]);
      //ROS_INFO("Read: %d",get_num);
      
      if(temp.size()==0&&i==1)
      {
        ROS_ERROR_STREAM("Read Overtime you can increase 'WaitDataTime' in launch file ");
        HadOvertime++;
        return false;
      }
      else if(temp.size()>=14)
      {
          uint8_t buf[14];
          if(HadOvertime&&temp.size()>14)
          {
            ROS_INFO("Read last");
            for(;HadOvertime>0;HadOvertime--)
            {
              for(int i=0;i<14;i++)
              {
                buf[i] = temp.at(0);
                temp.erase(temp.begin());
              }
            }
          }
          for(int i=0;i<14;i++)
          {
            buf[i] = temp.at(0);
            temp.erase(temp.begin());
          }
          readtempdata.DatafromStream(buf,14);
          ROS_INFO("Read: %X %X %X %X %X %X %X %X %X %X %X %X %X %X",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13]);
          return true;
        }
    }

  }
  return false;
}
