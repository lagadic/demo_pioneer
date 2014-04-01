#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"


class Joy2twist
{
private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwistPioneer_;
  ros::Publisher  pubTwistBiclops_;
  ros::Subscriber subJoy_;
  ros::Subscriber subTwistPioneer_;
  ros::Subscriber subTwistBiclops_;

  geometry_msgs::Twist in_pioneer_cmd_vel_; // input cmd_vel
  geometry_msgs::Twist in_biclops_cmd_vel_; // input cmd_vel

  bool joystrick_drive_;

public:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void twistPioneerCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void twistBiclopsCallback(const geometry_msgs::Twist::ConstPtr& msg);
  Joy2twist(int argc, char**argv);
  virtual ~Joy2twist(){};
};

Joy2twist::Joy2twist(int argc, char**argv)
{
  joystrick_drive_ = true;
  subJoy_   = nh_.subscribe("joy", 1000, &Joy2twist::joyCallback, this);
  subTwistPioneer_ = nh_.subscribe("vs/pioneer/cmd_vel", 1000, &Joy2twist::twistPioneerCallback, this);
  pubTwistPioneer_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
  subTwistBiclops_ = nh_.subscribe("vs/biclops/cmd_vel", 1000, &Joy2twist::twistBiclopsCallback, this);
  pubTwistBiclops_ = nh_.advertise<geometry_msgs::Twist>("biclops/cmd_vel", 1000);
}

void Joy2twist::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  std::ostringstream strs;
  strs << std::endl << "axes: [";
  for(size_t i=0; i < msg->axes.size(); i++)
      strs << msg->axes[i] << ",";
  strs << "]" << std::endl;
  strs << "button: [";
  for(size_t i=0; i < msg->buttons.size(); i++)
      strs << msg->buttons[i] << ",";
  strs << "]" << std::endl;
  std::string str;
  str = strs.str();
  ROS_DEBUG("%s", str.c_str());

  // si bouton 4 (devant gauche) ou 5 (devant droite) activé
  //   out_cmd_vel = in_cmd_vel
  // sinon
  //   out_cmd_vel = vitesse joy
  //   axe 3 (horizontal) = vitesse angulaire
  //   axe 4 (vertical)   = vitesse lineaire sur x
  if (msg->buttons[4] || msg->buttons[5])
    joystrick_drive_ = false;
  else
    joystrick_drive_ = true;

  geometry_msgs::Twist out_pioneer_cmd_vel;
  geometry_msgs::Twist out_biclops_cmd_vel;
  if (! joystrick_drive_) {
    out_pioneer_cmd_vel = in_pioneer_cmd_vel_;
    out_biclops_cmd_vel = in_biclops_cmd_vel_;
  }
  else {
    out_pioneer_cmd_vel.angular.z = msg->axes[3];
    out_pioneer_cmd_vel.linear.x = msg->axes[4];
    out_biclops_cmd_vel.linear.x = 0;
    out_biclops_cmd_vel.linear.y = 0;
  }
  pubTwistPioneer_.publish(out_pioneer_cmd_vel);
  pubTwistBiclops_.publish(out_biclops_cmd_vel);
}

void Joy2twist::twistPioneerCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // memorize the command
  in_pioneer_cmd_vel_.angular = msg->angular;
  in_pioneer_cmd_vel_.linear  = msg->linear;
  // publish the command if joystick button pressed
  if (! joystrick_drive_) {
    geometry_msgs::Twist out_pioneer_cmd_vel;
    out_pioneer_cmd_vel = in_pioneer_cmd_vel_;
    pubTwistPioneer_.publish(out_pioneer_cmd_vel);
  }
}
void Joy2twist::twistBiclopsCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // memorize the command
  in_biclops_cmd_vel_.angular = msg->angular;
  in_biclops_cmd_vel_.linear  = msg->linear;
  // publish the command if joystick button pressed
  if (! joystrick_drive_) {
    geometry_msgs::Twist out_biclops_cmd_vel;
    out_biclops_cmd_vel = in_biclops_cmd_vel_;
    pubTwistBiclops_.publish(out_biclops_cmd_vel);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TeleopPioneerPan");

  Joy2twist joy2twist(argc, argv);

  ros::spin();
}

