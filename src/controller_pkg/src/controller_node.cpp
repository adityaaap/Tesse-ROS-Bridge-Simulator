#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2019
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  Implementation of a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf2 provides useful functions to convert tf2
// messages to eigen types and vice versa.
#include <tf2_eigen/tf2_eigen.h>

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                                 end part 0
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class controllerNode{
  ros::NodeHandle nh;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

  ros::Publisher prop_speed_pub;
  ros::Subscriber desiredState_sub;
  ros::Subscriber currentState_sub;
  ros::Timer timer;


  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical
  double m;              // mass 
  double g;              // gravity
  double d;              // distance 
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position 
  Eigen::Vector3d v;     // current velocity 
  Eigen::Matrix3d R;     // current orientation
  Eigen::Vector3d omega; // current angular velocity

  // Desired state
  Eigen::Vector3d xd;    // desired position
  Eigen::Vector3d vd;    // desired velocity 
  Eigen::Vector3d ad;    // desired acceleration
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop


  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 2 |  Initialize ROS callback handlers
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // In this section, you need to initialize your handlers from part 1.
      // Specifically:
      //  - bind controllerNode::onDesiredState() to the topic "desired_state"
      //  - bind controllerNode::onCurrentState() to the topic "current_state"
      //  - bind controllerNode::controlLoop() to the created timer, at frequency
      //    given by the "hz" variable
    
      desiredState_sub = nh.subscribe("desired_state", 10, &controllerNode::onDesiredState, this); // 10 is the squeze size
      currentState_sub = nh.subscribe("current_state", 1, &controllerNode::onCurrentState, this);

      ros::Rate timer_frequency(hz);
      timer = nh.createTimer(timer_frequency, &controllerNode::controlLoop, this);

      prop_speed_pub = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 2);


      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  Tune your gains!

      // Change them in the provided launch file.
      nh.getParam("kx", kx);
      nh.getParam("kv", kv);
      nh.getParam("kr", kr);
      nh.getParam("komega", komega);
      ROS_INFO("Gain values:\nkx: %f \nkv: %f \nkr: %f \nkomega: %f\n", kx, kv, kr, komega);


      // Initialize constants
      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
      double a = cf*d/sqrt(2);
      F2W << cf, cf, cf, cf, 
             a,   a, -a, -a, 
             -a,  a, a, -a, 
             cd, -cd, cd, -cd;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  Objective: fill in xd, vd, ad, yawd

      geometry_msgs::Vector3 position = des_state.transforms[0].translation;
      geometry_msgs::Quaternion rotation = des_state.transforms[0].rotation;
      geometry_msgs::Vector3 velocity = des_state.velocities[0].linear;
      geometry_msgs::Vector3 acceleration = des_state.accelerations[0].linear;

      // fill in vectors
      xd << position.x, position.y, position.z;

      vd << velocity.x, velocity.y, velocity.z;

      ad << acceleration.x, acceleration.y, acceleration.z;


      //
      // Extract the yaw component from the quaternion in the incoming ROS
      //     message and store in the yawd class member variable

      tf2::Quaternion quaternion;
      tf2::fromMsg(rotation, quaternion);

      yawd = tf2::getYaw(quaternion);

  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Objective: fill in x, v, R and omega
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // Get the current position and velocity from the incoming ROS message and
      // fill in the class member variables x, v, R and omega accordingly.

      geometry_msgs::Point position = cur_state.pose.pose.position;
      geometry_msgs::Quaternion attitude = cur_state.pose.pose.orientation;

      geometry_msgs::Vector3 velocity = cur_state.twist.twist.linear;
      geometry_msgs::Vector3 ang_vel_in_world = cur_state.twist.twist.angular;
      

      // fill in vectors
      x << position.x, position.y, position.z;
      v << velocity.x, velocity.y, velocity.z;

      // omega
      Eigen::Vector3d angular_world_eigen;
      angular_world_eigen << ang_vel_in_world.x, ang_vel_in_world.y, ang_vel_in_world.z;

      Eigen::Quaterniond q;
      //tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
      tf2::fromMsg(cur_state.pose.pose.orientation, q);
      R = q.toRotationMatrix();
      // Rotate omega to body frame
      omega = R.transpose() * angular_world_eigen;
  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  Objective: Implement the controller!

    ex = x - xd;
    ev = v - vd;
    //
    // ~~~~ end solution

    // Compute the Rd matrix.
    //
    // 
    //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    //    - check out [1] fig. (3) for the remaining axes [use cross product]
    //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    //

    //      The z-axes are flipped, which affects the signs of:
    //         i) the gravity term and
    //        ii) the overall sign (in front of the fraction) in equation (12)
    //            of the paper
    //    - normalize axes!
    //

    Eigen::Vector3d b3d = (-kx * ex - kv * ev + m * g * e3 + m * ad);
    b3d.normalize();

    // Find b1d, b2d
    Eigen::Vector3d xwc, b1d, b2d;
    xwc << cos(yawd), sin(yawd), 0; // unit vector in b1d direction -> ultimately treated at b1d

    b2d = b3d.cross(xwc).normalized();
    b1d = b2d.cross(b3d).normalized();

    // Build Rd
    Eigen::Matrix3d Rd;
    Rd << b1d, b2d, b3d;
  
    //
    // Compute the orientation error (er) and the rotation-rate error (eomega)


    er = 0.5*Vee( (Rd.transpose() * R - R.transpose() * Rd));
    eomega = omega; // The (R.T * Rd - R.T * Rd)V term has negligible effect so not considered 
    

    double f = (-kx * ex - kv * ev + m * g * e3 + m * ad).dot(R*e3);
    Eigen::Vector3d M = -kr * er - komega * eomega + omega.cross(J * omega);


    // Recover the rotor speeds from the wrench computed above
    //
    //  Hints:
    //     - [1] eq. (1)
    //
    // CAVEATs:
    //     - we have different conventions for the arodynamic coefficients,
    //       Namely: C_{\tau f} = c_d / c_f
    //               (LHS paper [1], RHS our conventions [lecture notes])
    //
    //     - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    //       between b1 and b2). To resolve this, check out equation 6.9 in the
    //       lecture notes!
    //
    //     - The thrust forces are **in absolute value** proportional to the
    //       square of the propeller speeds. Negative propeller speeds - although
    //       uncommon - should be a possible outcome of the controller when
    //       appropriate. Note that this is the case in unity but not in real
    //       life, where propellers are aerodynamically optimized to spin in one
    //       direction!
    //
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***
    Eigen::Matrix<double, 4, 1> wrench;
    wrench << f, M;
    Eigen::Matrix<double, 4, 1> propv_sq = F2W.colPivHouseholderQr().solve(wrench);

    Eigen::Matrix<double, 4, 1> propv;
    for (int i=0; i<4; i++) {
      propv[i] = signed_sqrt( propv_sq[i] ); //propeller speeds are signed
    }


    if (isnan(propv[0]) || isnan(propv[1]) || isnan(propv[2]) || isnan(propv[3])) {
      ROS_WARN_THROTTLE(1, "One of the prop speeds is nan.");
    } else {
      mav_msgs::Actuators msg;
      msg.angular_velocities = {propv[0], propv[1], propv[2], propv[3]};
     
      prop_speed_pub.publish(msg);
      
    }
    //std::cout<< propv[0] << std::endl;
    ROS_INFO("propv: %f %f %f %f", propv[0], propv[1], propv[2], propv[3]);
    //
    // ~~~~ end solution
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  controllerNode n;
  ros::spin();
}


/******************************************************************************************************************/