#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>


#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace jacobian_control_ns{

class JacobianControlClass
{
public:
  
  // The current robot state (to get the time stamp)                                                                                                                              
  //pr2_mechanism_model::RobotState* robot_state_;

  // The chain of links and joints                                                                                                                                                
  KDL::Chain kdl_chain_;

  // KDL Solvers performing the actual computations                                                                                                                               
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // The variables (which need to be pre-allocated).                                                                                                                              
  KDL::JntArray  q_;            // Joint positions                                                                                                                                
  KDL::JntArray  q0_;           // Joint initial positions                                                                                                                        
  KDL::JntArrayVel  qdot_;      // Joint velocities                                                                                                                               
  KDL::JntArray  tau_;          // Joint torques                                                                                                                                  

  KDL::Frame     x_;            // Tip pose
  KDL::Frame     xo_;           // Object Pose      
  KDL::Twist     diff;                                                                                                                 
  KDL::Frame     xd_;           // Tip desired pose                                                                                                                               
  KDL::Frame     x0_;             // Tip initial pose                                                                                                                               

  KDL::Twist     xerr_;         // Cart error                                                                                                                                     
  KDL::Twist     xdot_;         // Cart velocity    
  KDL::Twist     xobj_;           // Object Velocity                                                                                                                              
  KDL::Wrench    F_;            // Cart effort                                                                                                                                    
  KDL::Jacobian  J_;            // Jacobian                                                                                                                                       

  // Note the gains are incorrectly typed as a twist,                                                                                                                             
  // as there is no appropriate type!                                                                                                                                             
  KDL::Twist     Kp_;           // Proportional gains                                                                                                                             
  KDL::Twist     Kd_;           // Derivative gains    

  // The trajectory variables                                                                                                                                                     
  double    circle_phase_;      // Phase along the circle                                                                                                                         
  ros::Time last_time_;         // Time of the last servo cycle                                                                                                                   

public:
  // Write functions here 
  void start();
  void kdl2eigen(KDL::Jacobian &J, Eigen::MatrixXd &e);
    void kdl2eigen(KDL::JntArray &q, Eigen::MatrixXd &e);
    void eigen2kdl(Eigen::MatrixXd &e, KDL::JntArray &q); 
    JacobianControlClass()
    {
      std::string root_name = "torso_lift_link";
      std::string tip_name= "r_wrist_roll_link";

      //get the robot URDF description
      std::string robot_param, robot_description;
      ros::NodeHandle nh("~");
      nh.searchParam("robot_description",robot_param);
      nh.param<std::string>(robot_param,robot_description,"");

      //chain_.init(robot, root_name, tip_name);

      //parse the tree
      KDL::Tree kdl_tree;
      kdl_parser::treeFromString(robot_description, kdl_tree);
      //get the chain
      kdl_tree.getChain(root_name, tip_name, kdl_chain_);

      //std::cout << "Joints: " << kdl_chain_.getNrOfJoints() << std::endl;
      //std::cout << "Segments: " << kdl_chain_.getNrOfSegments() << std::endl;


      // Construct the kdl solvers in non-realtime.                                                                                                                                   
      //chain_.toKDL(kdl_chain_);
      jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
      jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

      J_.resize(kdl_chain_.getNrOfJoints());
      xobj_.vel(0) = 0.0;
      xobj_.vel(1) = 0.0;
      xobj_.vel(2) = 0.0;


      x0_.p(0) = 1.5;
      x0_.p(1) = -0.2;
      x0_.p(2) = 0;
      //Tip:: 0.808282  -0.188194  0.0852824
      // Tip:: 0.806085  -0.188098  0.0951002

      xo_.p(0) = 0.0;
      xo_.p(1) = -0.188098;
      xo_.p(2) = 0.0951002;
    }
};
}