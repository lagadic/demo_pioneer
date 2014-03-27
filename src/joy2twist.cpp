#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Joy2twist
{
private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwist_;
  ros::Subscriber subJoy_;
  ros::Subscriber subTwist_;

  geometry_msgs::Twist in_cmd_vel_; // input cmd_vel

  bool joystrick_drive_;

public:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  Joy2twist(int argc, char**argv);
  virtual ~Joy2twist(){};
};

Joy2twist::Joy2twist(int argc, char**argv)
{
  joystrick_drive_ = true;
  subJoy_   = nh_.subscribe("joy", 1000, &Joy2twist::joyCallback, this);
  subTwist_ = nh_.subscribe("cmd_vel", 1000, &Joy2twist::twistCallback, this);
  pubTwist_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
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

  // Use Logitech wireless gamepad F710
  // if buton 4 (front left) or 5 (front right) active
  //   out_cmd_vel = in_cmd_vel
  // else
  //   out_cmd_vel = vitesse joy
  //   axe 3 (horizontal) = angular vel
  //   axe 4 (vertical)   = linera  vel along x
  if (msg->buttons[4] || msg->buttons[5])
    joystrick_drive_ = false;
  else
    joystrick_drive_ = true;

  geometry_msgs::Twist out_cmd_vel;
  if (! joystrick_drive_)
    out_cmd_vel = in_cmd_vel_;
  else {
    out_cmd_vel.angular.z = msg->axes[3];
    out_cmd_vel.linear.x = msg->axes[4];
  }
  pubTwist_.publish(out_cmd_vel);
}

void Joy2twist::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // memorize the command
  in_cmd_vel_.angular = msg->angular;
  in_cmd_vel_.linear  = msg->linear;
  // publish the command if joystick button pressed
  if (! joystrick_drive_) {
    geometry_msgs::Twist out_cmd_vel;
    out_cmd_vel = in_cmd_vel_;
    pubTwist_.publish(out_cmd_vel);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy2twist");

  Joy2twist joy2twist(argc, argv);

  ros::spin();
}

