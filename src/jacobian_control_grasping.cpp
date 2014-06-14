#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "jacobian_control_grasping/jacobian_control_grasping.h"
#include <math.h>


#define PI 3.14159265

using namespace jacobian_control_ns;

void JacobianControlClass::kdl2eigen(KDL::Jacobian &J, Eigen::MatrixXd &e)
{
  e.resize(J.rows(), J.columns());
  
  for(int i=0; i<int(J.rows()); i++)
  {
    for(int j=0; j<int(J.columns()); j++)
    {
      e(i,j) = J(i,j);
    }
  }
}

void JacobianControlClass::kdl2eigen(KDL::JntArray &q, Eigen::MatrixXd &e)
{
  e.resize(q.rows(), 1);
  for(int i=0; i<int(q.rows()); i++)
  {
    e(i,0) = q(i,0);
  }
}

void JacobianControlClass::eigen2kdl(Eigen::MatrixXd &e, KDL::JntArray &q) 
{ 
  q.resize(7);   
  for(int i=0; i<7; i++) 
    { 
      q(i,0) = e(i,0); 
    } 
}


int main(int argc, char** argv)
{

ros::init(argc, argv, "jacobian_control_grasping");
JacobianControlClass jacob;
jacob.q0_.resize(7);
jacob.q_.resize(7);

//std::cout << "000" << std::endl;

///// Initializing Initial Joint Positions and Initial tip location
// -0.0111938 4.71336 0.00097905 -2.8245 0.1033 -2.62535 0.00596594
jacob.q0_(0) = -0.785; jacob.q0_(1) = 0.436; jacob.q0_(2) = -1.57; jacob.q0_(3) = 1.169; 
jacob.q0_(4) = 0.0; jacob.q0_(5) =1.134; jacob.q0_(6) = 0.0;

// jacob.q0_(0) = -0.0111938; jacob.q0_(1) = 4.71336; jacob.q0_(2) = 0.00097905; jacob.q0_(3) = -2.8245; 
// jacob.q0_(4) = 0.1033; jacob.q0_(5) =-2.62535; jacob.q0_(6) = 0.00596594;
//std::cout << "111" << std::endl;

jacob.jnt_to_pose_solver_->JntToCart(jacob.q0_, jacob.x0_); // Initial tip location

//std::cout << "Initial Joint:: " <<  jacob.q0_(0) * 180/PI << " " << jacob.q0_(1) * 180/PI << " " <<jacob.q0_(2) * 180/PI << " " <<jacob.q0_(3) * 180/PI<< " " <<jacob.q0_(4) * 180/PI << " " <<
//jacob.q0_(5) * 180/PI << " " << jacob.q0_(6) * 180/PI << " " << std::endl;

//std::cout << "Tip:: " <<  jacob.x0_.p(0) << "  " <<  jacob.x0_.p(1) << "  " << jacob.x0_.p(2) <<  std::endl; 
////

jacob.q_ = jacob.q0_;
//std::cout << "222" << std::endl;

double dt =0.05;
double diff_gain = 0.5;
Eigen::MatrixXd iden = Eigen::MatrixXd::Identity(7,7); // Identity matrix of 7x7
Eigen::MatrixXd J_eigen(6,7); // Jacobian in eigen format 
Eigen::MatrixXd J_inverse(7,6); // Pseudo Jacobian Inverse
//std::cout << "1" << std::endl;

Eigen::VectorXd theta_h_dot(7);
theta_h_dot << 0,0,0,0,0,0,0; 
//double h = 10; 
//double limit_centers[7];
double joint_limits[7][2];
Eigen::MatrixXd q_e;
//double delta;
double angle;
double joint_range;
double deadband;
Eigen::MatrixXd theta_d_dot(7,1); 

//std::cout << "A" << std::endl;
joint_limits[0][0] = 40;
joint_limits[0][1] = -130; // r_shoulder_pan_joint

joint_limits[1][0] = 80;
joint_limits[1][1] = -30; // *_shoulder_lift_joint

joint_limits[2][0] = 44;
joint_limits[2][1] = -224; // r_upper_arm_roll_joint

joint_limits[3][0] = 133;
joint_limits[3][1] = 0; // *_elbow_flex_joint

joint_limits[4][0] = 0;
joint_limits[4][1] = 0; // *_forearm_roll_joint

joint_limits[5][0] = 130;
joint_limits[5][1] = 0; // *_wrist_flex_joint

joint_limits[6][0] = 0;
joint_limits[6][1] = 0; // *_wrist_roll_joint
        
 Eigen::VectorXd joint_velocity_limit(7);
joint_velocity_limit << 2.10,2.10,3.27,3.30,3.60,3.10,3.60;

for(int i = 0; i< 50;i++)
{
  // Compute the forward kinematics and Jacobian (at this location).                                                                                                              
    jacob.jnt_to_pose_solver_->JntToCart(jacob.q_, jacob.x_);
    jacob.jnt_to_jac_solver_->JntToJac(jacob.q_, jacob.J_);
    
    //std::cout << "Tip:: " <<  jacob.x_.p(0) << "  " <<  jacob.x_.p(1) << "  " << jacob.x_.p(2) <<  std::endl; 

    jacob.kdl2eigen(jacob.J_, J_eigen); // KDL::Jacobian to Eigen Matrix

    J_inverse = J_eigen.transpose()*((J_eigen*J_eigen.transpose()).inverse()); // Finding Jacobian Pseudo Inverse
    
    ////////////////////////////// Computng Joint Limiting avoiding velocities


    //jacob.kdl2eigen(jacob.q_, q_e);
    for (int i=0 ; i<7 ; i++)
    {
      joint_range = joint_limits[i][0] - joint_limits[i][1];
      deadband = 0.10 * joint_range;

      angle = jacob.q_(i) * 180 / PI;
      
      // if ((angle < joint_limits[i][0]) && (angle > joint_limits[i][0] - deadband))
      if (angle > joint_limits[i][0] - deadband)
        {
          theta_h_dot[i] = -0.5;
        }
      // if ((angle > joint_limits[i][1]) && (angle < joint_limits[i][1] + deadband))
      if (angle < joint_limits[i][1] + deadband)
        {
          theta_h_dot[i] = 0.5;
        }
      if (i==4 or i==6)
        theta_h_dot[i] = 0;
    }
    //////////////////////////////////

    //std::cout << "Vel: " << theta_h_dot(0,0) << "  " << theta_h_dot(1,0) << "  " << theta_h_dot(2,0) << "  " << theta_h_dot(3,0) << "  " << theta_h_dot(4,0) <<
    //"  " << theta_h_dot(5,0) << "  " << theta_h_dot(6,0) << "  " << std::endl; 
    
    ////////////////////////////////// Computing desired Task space velocities
  
    Eigen::MatrixXd theta_d_dot(7,1); 
    jacob.diff.vel = jacob.xo_.p - jacob.x_.p;
    
    jacob.xdot_.vel = diff_gain*jacob.diff.vel + jacob.xobj_.vel;
    
    Eigen::VectorXd xdot_vector(6); // desired Task space velocities
    //convert twist to vector
    xdot_vector << jacob.xdot_.vel(0),jacob.xdot_.vel(1),jacob.xdot_.vel(2),jacob.xdot_.rot(0),jacob.xdot_.rot(1),jacob.xdot_.rot(2);
    ///////////////////////////////

    
    Eigen::MatrixXd temp(7,7);
    temp = (iden - J_inverse*J_eigen);
    theta_d_dot = J_inverse*xdot_vector + (iden - J_inverse*J_eigen)*theta_h_dot; // desired Joint space velocities
    
    ////// Checking Joint space velocities limits
    // 2.10, 2.10, 3.27, 3.30, 3.60, 3.10, 3.60
    
   
    
    for (int i=0;i<7;i++)
    {
      if (fabs(theta_d_dot(i,0)) > joint_velocity_limit(i))
      {
        if (theta_d_dot(i,0) < 0)
          theta_d_dot(i,0) = -1.0*joint_velocity_limit(i);
        else
          theta_d_dot(i,0) = joint_velocity_limit(i);
      }

    }
    /////

    //std::cout << "Vel: " << theta_d_dot(0,0) << "  " << theta_d_dot(1,0) << "  " << theta_d_dot(2,0) << "  " << theta_d_dot(3,0) << "  " << theta_d_dot(4,0) <<
    //"  " << theta_d_dot(5,0) << "  " << theta_d_dot(6,0) << "  " << std::endl; 

    ////////////////////////////////// Updating the Joint Positions
    KDL::JntArray q_jnt;
    KDL::JntArray q_dest;
    
    jacob.eigen2kdl(theta_d_dot,q_jnt);
    KDL::Multiply(q_jnt,dt,q_jnt);
    KDL::Add(jacob.q_,q_jnt,q_dest);
    jacob.jnt_to_pose_solver_->JntToCart(q_dest, jacob.xd_);
    //////////////////////////////////

    jacob.q_ = q_dest;
    
    std::cout<<  jacob.q_(0) * 180/PI << " " << jacob.q_(1) * 180/PI << " " <<jacob.q_(2) * 180/PI  << " " <<jacob.q_(3) * 180/PI << " " <<jacob.q_(4) * 180/PI << " " <<
    jacob.q_(5) * 180/PI << " " << jacob.q_(6) * 180/PI << " " << std::endl; 


    
 }
// std::vector<double> angles;
// std::vector<double> spheres;
// cspace_->getCollisionCylinders(angles, spheres);

// *//visualization data
// int color = 180;
// int transparency = 0.5;
// std::string vis_name = "right_arm_model";

// *//construct visualizer*
// std::string group_name = "right_arm";

// boost::shared_ptr<sbpl_arm_planner::VisualizeArm> arm_vis;
// arm_vis = boost::make_shared<sbpl_arm_planner::VisualizeArm>(group_name);

// *//visualize the arm's collision model*
// arm_vis->visualizeSpheres(spheres, color, transparency, vis_name);
}

