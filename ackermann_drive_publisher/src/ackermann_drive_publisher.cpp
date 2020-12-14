#include <ackermann_drive_publisher/ackermann_drive_Config.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

class AckermannDrivePublisher {
 public:
  AckermannDrivePublisher();

 private:
  ros::NodeHandle nh;
  ros::Timer timer;
  ros::Publisher drive_pub;

  dynamic_reconfigure::Server<ackermann_drive_publisher::ackermann_drive_Config> server;
  dynamic_reconfigure::Server<ackermann_drive_publisher::ackermann_drive_Config>::CallbackType f;

  double target_speed;
  double target_acceleration;
  double target_jerk;
  double target_steering_angle;
  double target_steering_angle_velocity;

  void timerCallback(const ros::TimerEvent& event);
  void dynamicReconfigureCallback(ackermann_drive_publisher::ackermann_drive_Config& config, uint32_t level);
};

AckermannDrivePublisher::AckermannDrivePublisher() {
  ros::NodeHandle private_nh("~");

  std::string out_topic;

  double publish_rate;

  ROS_ASSERT(private_nh.getParam("publish_rate", publish_rate));
  ROS_ASSERT(private_nh.getParam("out_topic", out_topic));

  drive_pub = nh.advertise<ackermann_msgs::AckermannDrive>(out_topic, 1);
  timer = nh.createTimer(ros::Duration(1 / publish_rate), &AckermannDrivePublisher::timerCallback, this);

  f = boost::bind(&AckermannDrivePublisher::dynamicReconfigureCallback, this, _1, _2);
  server.setCallback(f);
}

void AckermannDrivePublisher::timerCallback(const ros::TimerEvent& event) {
  ackermann_msgs::AckermannDrive drive_msg;
  drive_msg.speed = target_speed;
  drive_msg.acceleration = target_acceleration;
  drive_msg.jerk = target_jerk;
  drive_msg.steering_angle = target_steering_angle;
  drive_msg.steering_angle_velocity = target_steering_angle_velocity;

  drive_pub.publish(drive_msg);
}

void AckermannDrivePublisher::dynamicReconfigureCallback(ackermann_drive_publisher::ackermann_drive_Config& config, uint32_t level) {
  target_speed = config.target_speed;
  target_acceleration = config.target_acceleration;
  target_jerk = config.target_jerk;
  target_steering_angle = config.target_steering_angle;
  target_steering_angle_velocity = config.target_steering_angle_velocity;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ackermann_drive_publisher_node");
  AckermannDrivePublisher node;
  ros::spin();
  return 0;
}
