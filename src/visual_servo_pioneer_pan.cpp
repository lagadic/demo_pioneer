#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <visp_bridge/3dpose.h>

#include <visp/vpAdaptiveGain.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDot.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPioneerPan.h>
#include <visp/vpRobot.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>

class VS
{
private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwistPioneer_; // cmd_vel
  ros::Publisher  pubTwistBiclops_; // cmd_vel
  ros::Subscriber subPoseTarget_;  // pose_stamped
  ros::Subscriber subStatusTarget_;  // pose_stamped
  ros::Subscriber subBiclopsOdom_;  // pose_stamped

  vpServo task;
  // Current and desired visual feature associated to the x coordinate of the point
  vpFeaturePoint s_x, s_xd;
  vpFeatureDepth s_Z, s_Zd;

  vpCameraParameters cam;
  double depth;
  double Z, Zd;
  double lambda;

  bool valid_pose;
  bool valid_pose_prev;

  double t_start_loop;
  double tinit;

  vpColVector v;
  vpColVector vi;
  vpColVector qm; // Biclops odom
  double qm_pan; // Measured pan position (tilt is not handled in that example)
  double mu;
  vpAdaptiveGain lambda_adapt;
  vpPioneerPan robot;

public:
  void init_vs();
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void biclopsOdomCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void statusCallback(const std_msgs::Int8ConstPtr& msg);
  VS(int argc, char**argv);
  virtual ~VS() {
    task.kill();
  };
};

VS::VS(int argc, char**argv)
{
  init_vs();

  subPoseTarget_   = nh_.subscribe("/visp_auto_tracker/object_position", 1000, &VS::poseCallback, this);
  subStatusTarget_ = nh_.subscribe("/visp_auto_tracker/status", 1000, &VS::statusCallback, this);
  subBiclopsOdom_   = nh_.subscribe("/biclops/odom", 1000, &VS::biclopsOdomCallback, this);
  pubTwistPioneer_  = nh_.advertise<geometry_msgs::Twist>("vs/pioneer/cmd_vel", 1000);
  pubTwistBiclops_  = nh_.advertise<geometry_msgs::Twist>("vs/biclops/cmd_vel", 1000);
}

void VS::init_vs()
{
  depth = 0.4;
  lambda = 1.;
  valid_pose = false;
  valid_pose_prev = false;

  Z = Zd = depth;

  v.resize(2);
  vi.resize(2);
  qm.resize(2);
  v = 0; vi = 0; qm = 0;
  mu = 4;
  qm_pan = 0;

  //lambda_adapt.initStandard(4, 0.5, 40);
  lambda_adapt.initStandard(3.5, 1.5, 15);

  cam.initPersProjWithoutDistortion(800, 795, 320, 216);

  task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;
  task.setLambda(lambda_adapt) ;

  vpVelocityTwistMatrix cVe = robot.get_cVe();
  // Update the robot jacobian that depends on the pan position
  robot.set_eJe(qm_pan);
  // Get the robot jacobian
  vpMatrix eJe = robot.get_eJe();
  task.set_eJe( eJe );

  vpImagePoint ip(0,0);

  // Create the current x visual feature
  vpFeatureBuilder::create(s_x, cam, ip);

  // Create the desired x* visual feature
  s_xd.buildFrom(0, 0, Zd);

  // Add the feature
  task.addFeature(s_x, s_xd, vpFeaturePoint::selectX()) ;

  s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0
  s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0

  // Add the feature
  task.addFeature(s_Z, s_Zd) ;

}

void VS::statusCallback(const std_msgs::Int8ConstPtr& msg)
{
  if (msg->data == 3)
    valid_pose = true;
  else
    valid_pose = false;
}

void VS::biclopsOdomCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::PoseStamped position;
  qm[1] = position.pose.orientation.x;
  qm[0] = position.pose.orientation.y;
}

void VS::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::Twist out_cmd_vel;
  geometry_msgs::Twist pioneer_cmd_vel;
  geometry_msgs::Twist biclops_cmd_vel;
  try {
    t_start_loop = vpTime::measureTimeMs();

    std::ostringstream strs;
    strs << "Receive a new pose" << std::endl;
    std::string str;
    str = strs.str();
    ROS_DEBUG("%s", str.c_str());

    vpHomogeneousMatrix cMo = visp_bridge::toVispHomogeneousMatrix(msg->pose);

    vpPoint origin;
    origin.setWorldCoordinates(0,0,0);
    origin.project(cMo);
    Z = origin.get_Z();

    if (Z <= 0)
      ROS_DEBUG("Z <= 0");

    if (! valid_pose || Z <= 0) {
      ROS_DEBUG("not valid pose");

      out_cmd_vel.linear.x = 0;
      out_cmd_vel.linear.y = 0;
      out_cmd_vel.linear.z = 0;
      out_cmd_vel.angular.x = 0;
      out_cmd_vel.angular.y = 0;
      out_cmd_vel.angular.z = 0;
      pubTwistPioneer_.publish(out_cmd_vel);
      pubTwistBiclops_.publish(out_cmd_vel);

      valid_pose = false;
      valid_pose_prev = valid_pose;

      return;
    }

    // Update the current x feature
    s_x.set_xyZ(origin.p[0], origin.p[1], Z);

    // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd)) ;

    vpVelocityTwistMatrix cVe = robot.get_cVe();
    task.set_cVe( cVe );
    
      // Update the robot jacobian that depends on the pan position
      robot.set_eJe(qm_pan);
      // Get the robot jacobian
      vpMatrix eJe = robot.get_eJe();
      // Update the jacobian that will be used to compute the control law
      task.set_eJe(eJe);

    // Compute the control law. Velocities are computed in the mobile robot reference frame
    v = task.computeControlLaw() ;

    static unsigned long iter = 0;
//    if (valid_pose_prev == false) {
    if (iter == 0) {
      // Start a new visual servo
      ROS_INFO("Reinit visual servo");

      tinit = t_start_loop;
      vi = v;
    }
    iter ++;

    //v = v - vi*exp(-mu*(t_start_loop - tinit)/1000.);
    double max_linear_vel = 0.5;
    double max_angular_vel = vpMath::rad(50);
    vpColVector v_max(3);
    v_max[0] = max_linear_vel;
    v_max[1] = max_angular_vel;
    v_max[2] = max_angular_vel;

    vpColVector v_sat = vpRobot::saturateVelocities(v, v_max);

//    if (std::abs(v[0]) > max_linear_vel || std::abs(v[1]) > max_angular_vel || std::abs(v[2]) > max_angular_vel) {
//      ROS_INFO("Vel exceed max allowed");
//      for (unsigned int i=0; i< v.size(); i++)
//        ROS_INFO("v[%d]=%f", i, v[i]);
//      v = 0;
//    }

 
    pioneer_cmd_vel.linear.x = v_sat[0];
    pioneer_cmd_vel.linear.y = 0;
    pioneer_cmd_vel.linear.z = 0;
    pioneer_cmd_vel.angular.x = 0;
    pioneer_cmd_vel.angular.y = 0;
    pioneer_cmd_vel.angular.z = v_sat[1];

    biclops_cmd_vel.linear.x = 0;
    biclops_cmd_vel.linear.y = 0;
    biclops_cmd_vel.linear.z = 0;
    biclops_cmd_vel.angular.x = 0;
    biclops_cmd_vel.angular.y = v_sat[2];
    biclops_cmd_vel.angular.z = 0;

    pubTwistPioneer_.publish(pioneer_cmd_vel);
    if (t_start_loop - tinit>2000)
      pubTwistBiclops_.publish(biclops_cmd_vel);
    valid_pose_prev = valid_pose;

    valid_pose = false;
  }
  catch(...) {
    ROS_INFO("Catch an exception: set vel to 0");
    out_cmd_vel.linear.x = 0;
    out_cmd_vel.linear.y = 0;
    out_cmd_vel.linear.z = 0;
    out_cmd_vel.angular.x = 0;
    out_cmd_vel.angular.y = 0;
    out_cmd_vel.angular.z = 0;
    pubTwistPioneer_.publish(out_cmd_vel);
    pubTwistBiclops_.publish(out_cmd_vel);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PioneerPan");

  VS vs(argc, argv);

  ros::spin();
}


