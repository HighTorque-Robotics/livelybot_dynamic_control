include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <iostream>
#include <ros/ros.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/xxx/workspace/xxx/src/xxxxx_urdf"
#endif
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Pinocchio_test");
  ros::NodeHandle nh;
  using namespace pinocchio;

  // You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/urdf/xxx_urdf.urdf") : argv[1];

  // Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  std::cout << "model name: " << model.name << std::endl;

  // Create data required by the algorithms
  Data data(model);

  // Sample a random configuration
//  Eigen::VectorXd q= randomConfiguration(model);
  Eigen::VectorXd q(model.nq) ,v(model.nv),a(model.nv),tau(model.nq);
  Eigen::Matrix<double,6,4> J1,J2,dJ;//the model has 4 joints
  q<<0,0,0,0;
  v<<0,0.1,0.1,0;
  a<<0,0,0,0;
  J1<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
  J2=J1;
  dJ=J1;
  std::cout << "q: " << q.transpose() << std::endl;
  std::cout << "v: " << v.transpose() << std::endl;
  std::cout << "a: " << a.transpose() << std::endl;


* Perform the forward kinematics over the kinematic tree *
  forwardKinematics(model,data,q,v,a);
  // Print out the state vector/matrix of each joint of the kinematic tree
  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "<<std::endl
              << std::fixed << std::setprecision(6)
              << data.liMi[joint_id].translation().transpose()<<std::endl /*Vector : placements of the joint (relative to the body parent) expressed at the body parent.*/
              << data.liMi[joint_id].rotation()<<std::endl                /*Matrix : rotation transform matrix of the joint (relative to the body parent)*/
              << data.oMi[joint_id].translation().transpose()<<std::endl  /*Vector : placements of the joint (relative to the world) expressed at the world coordinate.*/
              << data.oMi[joint_id].rotation()<<std::endl                 /*Matrix : rotation transform matrix of the joint (relative to the world).*/
              << data.v[joint_id].linear()<<std::endl                     /*Vector : the velocities of the joint (relative to the world) expressed at the current joint coordinate.*/
              << data.v[joint_id].angular()<<std::endl                    /*Vector : the angular velocities of the joint (relative to the world) expressed at the current joint coordinate.*/
              << data.a[joint_id].linear()<<std::endl                     /*Vector : the acceleratetion of the joint (relative to the world) expressed at the current joint coordinate.(Not considered the centrifugal effect)*/
              << data.a[joint_id].angular()<<std::endl                    /*Vector : the angular acceleratetion of the joint (relative to the world) expressed at the current joint coordinate.(Not considered the centrifugal effect)*/
              << std::endl;

    /*the LOCAL means the velocity got from the function is the joint's velocity(relative to the world) expressed at the current joint coordinate.*/
    /*the WORLD means the velocity got from the function is the world coordinate's velocity(relative to the joint) expressed at the world coordinate.*/
    /*the LOCAL_WORLD_ALIGNED means the velocity got from the function is the joint's velocity(relative to the world) expressed at the world coordinate.*/
    std::cout << getVelocity(model,data,joint_id,LOCAL_WORLD_ALIGNED).linear()<<std::endl
              << getVelocity(model,data,joint_id,LOCAL_WORLD_ALIGNED).angular()<<std::endl
              << std::endl;

    /*there are two functions could get the acceleration of joint,the getAcceleration() dosen't consider the centrifugal effect(离心效应), and the getClassicalAcceleration() does, so the last one is useful.*/
    /*the LOCAL means the acceleration got from the function is the joint's acceleration(relative to the world) expressed at the current joint coordinate.*/
    /*the WORLD means the acceleration got from the function is the world coordinate's acceleration(relative to the joint) expressed at the world coordinate.*/
    /*the LOCAL_WORLD_ALIGNED means the acceleration got from the function is the joint's acceleration(relative to the world) expressed at the world coordinate.*/
    std::cout << getClassicalAcceleration(model,data,joint_id,LOCAL_WORLD_ALIGNED).linear()<<std::endl
              << getClassicalAcceleration(model,data,joint_id,LOCAL_WORLD_ALIGNED).angular()<<std::endl
              << std::endl;
  }


* Perform the kinematic jacobian *
//  forwardKinematics(model,data,q,v,a);//for test
  computeJointJacobians(model,data,q);//You have to run pinocchio::computeJointJacobians() before calling pinocchio::getJointJacobian();The function also compute the forwared kinematic.
  computeJointJacobiansTimeVariation(model,data,q,v);//You have to run pinocchio::computeJointJacobiansTimeVariation*() before calling pinocchio::getJointJacobianTimeVariation().
  // Print out the jacobian matrix of each joint
  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "<<std::endl
              << std::fixed << std::setprecision(6)
              << std::endl;

    /*the LOCAL means the velocity got from the function is the joint's velocity(relative to the world) expressed at the current joint coordinate.*/
    /*the WORLD means the velocity got from the function is the world coordinate's velocity(relative to the joint) expressed at the world coordinate.*/
    /*the LOCAL_WORLD_ALIGNED means the velocity got from the function is the joint's velocity(relative to the world) expressed at the world coordinate.*/
    std::cout << getVelocity(model,data,joint_id,LOCAL_WORLD_ALIGNED).linear()<<std::endl
              << getVelocity(model,data,joint_id,LOCAL_WORLD_ALIGNED).angular()<<std::endl
              << std::endl;

    /* computeJointJacobian() function: Computes the Jacobian (relative to the world) of a specific joint frame expressed in the local frame of the joint and store the result in the input argument J*/
    computeJointJacobian(model,data,q,joint_id,J1);
    std::cout<<J1*v<<std::endl<<std::endl;                //the matrix J1 stored the results of the Jacobian; J1*v means the joint's velocity (relative to the world) expressed in the local joint frame.

    /* getJointJacobian() function: Computes the Jacobian (relative to the world) of a specific joint frame expressed either in the world (rf = LOCAL_WORLD_ALIGNAED) frame or in the local frame (rf = LOCAL) of the joint*/
    //You have to run pinocchio::computeJointJacobians() before calling getJointJacobian();The function also compute the forwared kinematic.
    /*the LOCAL means the result got from the function is the joint's Jacobian (relative to the world) expressed at the current joint coordinate.(like the getVelocity() function)*/
    /*the WORLD means the result got from the function is the world coordinate's jacobian (relative to the joint) expressed at the world coordinate.(like the getVelocity() function)*/
    /*the LOCAL_WORLD_ALIGNED means the result got from the function is the joint's jacobian (relative to the world) expressed at the world coordinate.(like the getVelocity() function)*/
    getJointJacobian(model,data,joint_id,LOCAL_WORLD_ALIGNED,J2);
    std::cout<<J2*v<<std::endl<<std::endl;                //the matrix J2 stored the results of the Jacobian;
    ///\note:The result to call first computeJointJacobians(model,data,q) and then call getJointJacobian(model,data,jointId,LOCAL,J) is equivalentof to call computeJointJacobian() ,
    ///      but only call the computeJointJacobian() function ,the forwardKinematics is not fully computed.
    ///      It is worth to call jacobian if you only need a single Jacobian for a specific joint. Otherwise, for several Jacobians, it is better
    ///      to call computeJointJacobians(model,data,q) followed by getJointJacobian(model,data,jointId,LOCAL,J) for each Jacobian.

    /* getJointJacobianTimeVariation() function: Computes the Jacobian time variation of a specific joint frame expressed either in the world frame (rf = LOCAL_WORLD_ALIGNED) or in the local frame (rf = LOCAL) of the joint. */
    //You have to run pinocchio::computeJointJacobiansTimeVariation*() before calling pinocchio::getJointJacobianTimeVariation().
    getJointJacobianTimeVariation(model,data,joint_id,LOCAL_WORLD_ALIGNED,dJ);
    std::cout<<dJ<<std::endl;
  }


* Perform the Inverse dynamics based on the recursive Newton-Euler Algorithm(RNEA)*
* M(q)*d2q/dt2+C(q,dq/dt)*dq/dt/+G(q)=tau+sum(J^T(q)*fext) *


  /* compute the joint torque according the state(q,v,a)*/
  /* data.tau=M(q)*d2q/dt2+C(q,dq/dt)*dq/dt/+G(q) */
  q<<0,0,0,0;
  v<<0,0,0,0;
  a<<0,0,0,0;
  rnea(model,data,q,v,a);
  std::cout<<data.tau<<std::endl<<std::endl;//the tau store the torque (expressed in the local coordinate revolute axis) that should be loaded on the joints to keep the state without external force/torque.
  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "<<std::endl
              << std::fixed << std::setprecision(6)
              <<data.f[joint_id]<<std::endl/*before call the rnea(//with exrtern force), the data.f only store the force & torque from gravity of bodies,the direction of the force is inverse*/
              <<data.f[joint_id].linear()<<std::endl
              <<data.f[joint_id].angular()<<std::endl
              << std::endl;
    //the f stores the force and torque that gravity acting on the body, but expressed in the local body coordinate of inverse direction.
  }


  /* compute the joint torque according the state(q,v,a) and the external force & torque */
  /* data.tau=M(q)*d2q/dt2+C(q,dq/dt)*dq/dt/+G(q)-sum(J^T(q)*fext) */
  Eigen::Vector3d L0(0,0,0),L1(0,0,0),L2(0,0,0),L3(0,0,0),L4(0,1,0);//the external forces are expressed in the local body/joint coordinate.
  Eigen::Vector3d A0(0,0,0),A1(0,0,0),A2(0,0,0),A3(0,0,0),A4(0,0,0);
  Force F0(L0,A0),F1(L1,A1),F2(L2,A2),F3(L3,A3),F4(L4,A4);
  PINOCCHIO_ALIGNED_STD_VECTOR(Force) extf(5);
  PINOCCHIO_ALIGNED_STD_VECTOR(Force)::iterator it=extf.begin();
  *it=F0;it++;
  *it=F1;it++;
  *it=F2;it++;
  *it=F3;it++;
  *it=F4;
  rnea(model,data,q,v,a,extf);
  std::cout<<data.tau<<std::endl<<std::endl;//the tau store the torque (expressed in the local coordinate revolute axis) that should be loaded on the joints to keep the state with the external force/torque.
  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
  {
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "<<std::endl
              << std::fixed << std::setprecision(6)
              <<data.f[joint_id]<<std::endl/*after call the rnea(//with exrtern force), the data.f store all the external force & torque(include the gravity),the direction of the force is inverse */
              <<extf[joint_id].linear()<<std::endl
              <<extf[joint_id].angular()<<std::endl
              <<extf[joint_id]<<std::endl
              << std::endl;
    //the f stores the force and torque that all external forces & torques acting on the body, but expressed in the local body coordinate of inverse direction.
  }


  /* Computes the non-linear effects (Corriolis, centrifual and gravitationnal effects) */
  /* This function is equivalent to pinocchio::rnea(model, data, q, v, 0). */
  /* data.nle=b(q,dq/dt)=C(q,dq/dt)*dq/dt/+G(q) */
  nonLinearEffects(model,data,q,v);
  std::cout<<data.nle<<std::endl
           <<std::endl;


  /* Computes the generalized gravity contribution G(q)  of the Lagrangian dynamics */
  /* This function is equivalent to pinocchio::rnea(model, data, q, 0, 0). */
  /* data.g=G(q) */
  computeGeneralizedGravity(model,data,q);
  std::cout<<data.g<<std::endl
           <<std::endl;


  /* Computes the generalized gravity contribution G(q)-sum(J^T(q)*fext) of the Lagrangian dynamics */
  /* This function is equivalent to pinocchio::rnea(model, data, q, 0, 0, fext). */
  /* data.tau=G(q)-sum(J^T(q)*fext) */
  computeStaticTorque(model,data,q,extf);
  std::cout<<data.tau<<std::endl
           <<std::endl;


  /* Computes the Coriolis Matrix of the Lagrangian dynamics */
  /* data.C=C(q,dq/dt) */
  computeCoriolisMatrix(model,data,q,v);
  std::cout<<data.C<<std::endl
           <<std::endl;

  /* return the Coriolis Matrix C(q,dq/dt) of the Lagrangian dynamics after a call to the dynamics derivatives.*/
  /* data.C=C(q,dq/dt) */
  getCoriolisMatrix(model,data);
  std::cout<<data.C<<std::endl
           <<std::endl;



}
————————————————

                            版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
                        
原文链接：https://blog.csdn.net/Mr_Yu_1997/article/details/119736667