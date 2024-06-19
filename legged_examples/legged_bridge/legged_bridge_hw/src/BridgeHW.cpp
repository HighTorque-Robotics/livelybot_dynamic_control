#include "legged_bridge_hw/BridgeHW.h"
#include "std_msgs/Float64MultiArray.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <cmath>

namespace legged
{
  
void writedata2file(float pos,float vel,float tau,std::string path)
{
  std::ofstream file(path, std::ios::app);
  if (!file.is_open()) {
        std::cerr << "无法打开文件用于写入\n";
        return;
    }
  file << pos << " "<<vel<<" "<<tau<<"\n";
  file.close();
}
bool BridgeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{

  root_nh.setParam("gsmp_controller_switch", "null");

  odom_sub_ = root_nh.subscribe("/imu/data", 1, &BridgeHW::OdomCallBack, this);
  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", powerLimit_);
   
  setupJoints();
  setupImu();
  cmd_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_pos", 10);
  cmd_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_vel", 10);
  cmd_ff_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_ff", 10);

  read_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_pos", 10);
  read_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_vel", 10);
  read_ff_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_ff", 10);  
  
  start = true;

  return true;
}


void BridgeHW::read(const ros::Time &time, const ros::Duration &period)
{
  // std_msgs::Float64MultiArray read_pos_msg, read_vel_msg, read_ff_msg;
  // read_pos_msg.data.resize(10);
  // read_vel_msg.data.resize(10);
  // read_ff_msg.data.resize(10);  
  float pos,vel,tau;
  for (int i=0; i<10; i++){
    // motor *m = motorsInterface->Motors[map_index_12dof[i]];
    // motor_back_t motor_data;
    // motor_data = *m->get_current_motor_state();

    // if(i==3||i==4){
    // jointData_[i].pos_ = 0.0;
    // jointData_[i].vel_ = 0.0;
    // jointData_[i].tau_ = 0.0;
    // }
    // else{
    motorsInterface->get_motor_state_dynamic_config(pos,vel,tau,map_index_12dof[i]);
    test_jointData_[i].pos_ = pos;
    test_jointData_[i].vel_ = vel;
    test_jointData_[i].tau_ = tau;  //电机真实数据
    // if(i == 4 || i == 9){
    //   jointData_[i].pos_ = -pos;
    //   jointData_[i].vel_ = -vel;
    //   jointData_[i].tau_ = -tau;
    // }
    
  }
  
  
  for(int i=0;i<10;i++)
  {
    if(i==3 || i==8)
    {
      jointData_[i].pos_ = test_directionMotor_[i] * test_jointData_[i].pos_ - test_directionMotor_[i-1] * test_jointData_[i-1].pos_;
      jointData_[i].vel_ = test_directionMotor_[i] * test_jointData_[i].vel_ - test_directionMotor_[i-1] * test_jointData_[i-1].vel_;
    }
    else 
    {
      jointData_[i].pos_ = test_directionMotor_[i] * test_jointData_[i].pos_;
      jointData_[i].vel_ = test_directionMotor_[i] * test_jointData_[i].vel_;
    }

    if(i==2 || i==7)
    {
      jointData_[i].tau_ = test_directionMotor_[i] * test_jointData_[i].tau_ + test_directionMotor_[i+1] * test_jointData_[i+1].tau_;
    }
    else 
    {
      jointData_[i].tau_ = test_directionMotor_[i] * test_jointData_[i].tau_;
    }    
    // read_pos_msg.data[i] = test_jointData_[i].pos_;
    // read_vel_msg.data[i] = test_jointData_[i].vel_;
  }
  
  // std::cerr<<"右手系下大腿电机力矩："<<jointData_[2].tau_<<std::endl;
  // std::cerr<<"右手系下小腿电机力矩："<<jointData_[3].tau_<<std::endl;
  // read_pos_pub_.publish(read_pos_msg);
  // read_vel_pub_.publish(read_vel_msg);

  // jointData_[3].pos_ = jointData_[3].pos_ + jointData_[2].pos_;
  // jointData_[3].vel_ = jointData_[3].vel_ + jointData_[2].vel_;
  // // jointData_[3].tau_ = jointData_[3].tau_ + jointData_[2].tau_;

  // jointData_[8].pos_ = jointData_[8].pos_ + jointData_[7].pos_;
  // jointData_[8].vel_ = jointData_[8].vel_ + jointData_[7].vel_;
  // jointData_[8].tau_ = jointData_[8].tau_ + jointData_[7].tau_;
  
  // for(int i=0;i<10;i++)
  // {
  //   jointData_[i].pos_ = jointData_[i].pos_ * directionMotor_[i];
  //   jointData_[i].vel_ = jointData_[i].vel_ * directionMotor_[i];
  //   jointData_[i].tau_ = jointData_[i].tau_ * directionMotor_[i];
  // }

  imuData_.ori[0] = yesenceIMU_.orientation.x;       
  imuData_.ori[1] = yesenceIMU_.orientation.y; 
  imuData_.ori[2] = yesenceIMU_.orientation.z; 
  imuData_.ori[3] = yesenceIMU_.orientation.w; 
  imuData_.angular_vel[0] = yesenceIMU_.angular_velocity.x;  
  imuData_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
  imuData_.angular_vel[2] = yesenceIMU_.angular_velocity.z;
  imuData_.linear_acc[0] = yesenceIMU_.linear_acceleration.x;   
  imuData_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
  imuData_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;

  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto &name : names)
  {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.0);
    handle.setKp(0.);
  }


}

void BridgeHW::write(const ros::Time& time, const ros::Duration& period)
{
  // std_msgs::Float64MultiArray cmd_pos_msg, cmd_vel_msg, cmd_ff_msg;

  // cmd_pos_msg.data.resize(10);
  // cmd_vel_msg.data.resize(10);
  // cmd_ff_msg.data.resize(10);
  
  // if(start == true)
  // {
  //   start = false;
  //   begin = ros::Time::now();
  // }

  // ros::Time time_now = ros::Time::now();
  
  // double t = (time_now - begin).toSec();




  for (int i = 0; i < 10; ++i)//as the urdf rank
  {
    // yksSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * test_directionMotor_[i] ;//+ write_baseMotor_[i] ;
    // yksSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * test_directionMotor_[i];
    yksSendcmd_[i].kp_ = jointData_[i].kp_;
    yksSendcmd_[i].kd_ = jointData_[i].kd_;
    // yksSendcmd_[i].ff_ = jointData_[i].ff_ * test_directionMotor_[i] * 1; 
    
    // if(i==2 || i==3)
    // {
    //   yksSendcmd_[i].kp_ = 0.0;
    //   yksSendcmd_[i].kd_ = 0.0;
    // }

    if(i==3 || i==8)
    {
      yksSendcmd_[i].pos_des_ = test_directionMotor_[i]*(jointData_[i].pos_des_ + jointData_[i-1].pos_des_);
      yksSendcmd_[i].vel_des_ = test_directionMotor_[i]*(jointData_[i].vel_des_ + jointData_[i-1].vel_des_);
    }
    else 
    {
      yksSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * test_directionMotor_[i];//+ write_baseMotor_[i] ;
      yksSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * test_directionMotor_[i];
    }

    if(i==2 || i==7)
    {
      yksSendcmd_[i].ff_ = test_directionMotor_[i]*(jointData_[i].ff_ - jointData_[i+1].ff_) * 1;
      // if(i==2)
      // {
      //   // yksSendcmd_[i].ff_ = test_directionMotor_[i]*(0 - 1*sin(2*t));
      //   yksSendcmd_[i].ff_ = test_directionMotor_[i]*(0-1.5);
      // }
    }
    else if(i==1 || i==6)
    {

      yksSendcmd_[i].ff_ = test_directionMotor_[i]*jointData_[i].ff_ * 1;
      // if(i==3)
      // {
      //   // yksSendcmd_[i].ff_ = test_directionMotor_[i]*(1*sin(2*t));
      //   yksSendcmd_[i].ff_ = test_directionMotor_[i]*(1.5);
      // }
    }
    else 
    {
      yksSendcmd_[i].ff_ = test_directionMotor_[i]*jointData_[i].ff_ * 1;
    }
    
    
    // if(i==1 || i==6)
    // {
    //   yksSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_[i] * 0.7;
    // }
    // else
    // {
    //   yksSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_[i] * 1;
    // }

    
    
    // if(i == 2 || i == 7 || i == 3 || i == 8){
    //   yksSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_[i] * 1;
    // }
    // if(i == 1 || i == 6){
    //       yksSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_[i] * 1;
    // }    
    // if(i==4 || i == 9)
    // {
    //   yksSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_[i] * 1;
    // }
    // if(i==0 || i==5)
    // {
    //   yksSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_[i] * 1;
    // }
  }
  

  // for (int i = 0; i < 10; ++i)//as directionMotor_the urdf rank
  // {
  //   cmd_pos_msg.data[i] = yksSendcmd_[i].pos_des_;
  //   cmd_vel_msg.data[i] = yksSendcmd_[i].vel_des_;
  //   cmd_ff_msg.data[i] = yksSendcmd_[i].ff_;
  // }
  // cmd_pos_pub_.publish(cmd_pos_msg);
  // cmd_vel_pub_.publish(cmd_vel_msg);
  // cmd_ff_pub_.publish(cmd_ff_msg);
  // std::cout<<std::endl;

  for (int i = 0; i < 10; ++i){
    // motor *m = motorsInterface->Motors[map_index_12dof[i]];
    // if(i==2 || i==7)  //大腿电机
    // {
    //   motorsInterface->fresh_cmd_dynamic_config(yksSendcmd_[i].pos_des_, yksSendcmd_[i].vel_des_,yksSendcmd_[i].ff_ - yksSendcmd_[i+1].ff_, yksSendcmd_[i].kp_, yksSendcmd_[i].kd_,map_index_12dof[i]);
    // }
    // else if(i == 3 || i == 8)
    // {
    //   motorsInterface->fresh_cmd_dynamic_config(yksSendcmd_[i].pos_des_ - yksSendcmd_[i - 1].pos_des_, (yksSendcmd_[i].vel_des_ - yksSendcmd_[i - 1].vel_des_), yksSendcmd_[i].ff_ , yksSendcmd_[i].kp_, yksSendcmd_[i].kd_,map_index_12dof[i]);
    // }
    if(i==4 || i==9){
      motorsInterface->fresh_cmd_dynamic_config(yksSendcmd_[i].pos_des_, yksSendcmd_[i].vel_des_, yksSendcmd_[i].ff_ *0.5, yksSendcmd_[i].kp_, yksSendcmd_[i].kd_,map_index_12dof[i]);

      //  m->fresh_cmd(yksSendcmd_[i].pos_des_, yksSendcmd_[i].vel_des_, std::clamp(yksSendcmd_[i].ff_ * 0,-3. , 3.), yksSendcmd_[i].kp_ , yksSendcmd_[i].kd_ );
    }
    else{
      motorsInterface->fresh_cmd_dynamic_config(yksSendcmd_[i].pos_des_, yksSendcmd_[i].vel_des_,yksSendcmd_[i].ff_ , yksSendcmd_[i].kp_, yksSendcmd_[i].kd_,map_index_12dof[i]);

        // m->fresh_cmd(yksSendcmd_[i].pos_des_, yksSendcmd_[i].vel_des_,yksSendcmd_[i].ff_*0, yksSendcmd_[i].kp_, yksSendcmd_[i].kd_);
    }

  }

  motorsInterface->fresh_cmd_dynamic_config(-yksSendcmd_[4].pos_des_, -yksSendcmd_[4].vel_des_, -yksSendcmd_[4].ff_*0.5 , yksSendcmd_[4].kp_, yksSendcmd_[4].kd_,0);
  motorsInterface->fresh_cmd_dynamic_config(-yksSendcmd_[9].pos_des_,-yksSendcmd_[9].vel_des_,  -yksSendcmd_[9].ff_*0.5 , yksSendcmd_[9].kp_, yksSendcmd_[9].kd_,6);
  // motor *left_tol = motorsInterface->Motors[0];
  // motor *right_tol = motorsInterface->Motors[6];
  // left_tol->fresh_cmd(-yksSendcmd_[4].pos_des_, -yksSendcmd_[4].vel_des_, -yksSendcmd_[4].ff_ * 0, yksSendcmd_[4].kp_, yksSendcmd_[4].kd_);
  // right_tol->fresh_cmd(-yksSendcmd_[9].pos_des_,-yksSendcmd_[9].vel_des_,  -yksSendcmd_[9].ff_ * 0, yksSendcmd_[9].kp_, yksSendcmd_[9].kd_);

  motorsInterface->motor_send_2();
  
}

bool BridgeHW::setupJoints()
{
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("leg_l") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (joint.first.find("leg_r") != std::string::npos)
    {
      leg_index = 1;
    }
    else
      continue;
    if (joint.first.find("1_joint") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("2_joint") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("3_joint") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("4_joint") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("5_joint") != std::string::npos)
      joint_index = 4;
    else
      continue;

    int index = leg_index * 5 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].pos_des_,
                                                           &jointData_[index].vel_des_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool BridgeHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link", imuData_.ori, imuData_.ori_cov, imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc, imuData_.linear_acc_cov));
  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;

  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  return true;
}

}  // namespace legged
