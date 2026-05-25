#include "abot_pid/abot_pid_core.h"

AbotPID::AbotPID()
{
}

AbotPID::~AbotPID()
{
}

void AbotPID::publishMessage(ros::Publisher *pub_message)
{
  riki_msgs::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void AbotPID::messageCallback(const riki_msgs::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void AbotPID::configCallback(abot_pid::abotPIDConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}
