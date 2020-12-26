#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class CarlaVehicleStatePublisher {
 public:
  CarlaVehicleStatePublisher();

  virtual ~CarlaVehicleStatePublisher() {}

 private:
  ros::NodeHandle nh;

  ros::Subscriber odom_sub;
  ros::Subscriber carla_vehicle_status_sub;

  ros::Publisher pose_pub;
  ros::Publisher twist_pub;
  ros::Publisher steering_angle_pub;

  std::string in_odom_topic;
  std::string carla_vehicle_status_topic;
  std::string twist_topic;
  std::string steering_angle_topic;

  double max_steering_angle;

  void odomCallback(const nav_msgs::Odometry in_odom_msg);
  void vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus vehicle_status_msg);

  geometry_msgs::Twist getBaseLinkTwist(geometry_msgs::Twist cog_twist);
};

CarlaVehicleStatePublisher::CarlaVehicleStatePublisher() {
  ros::NodeHandle private_nh("~");

  private_nh.param("in_odom_topic", in_odom_topic, std::string("/carla/ego_vehicle/odometry"));
  private_nh.param("carla_vehicle_status_topic", carla_vehicle_status_topic, std::string("/carla/ego_vehicle/vehicle_status"));
  private_nh.param("twist_topic", twist_topic, std::string("/vehicle/twist"));
  private_nh.param("steering_angle_topic", steering_angle_topic, std::string("/vehicle/steering_angle"));
  
  ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle));

  odom_sub = nh.subscribe(in_odom_topic, 1, &CarlaVehicleStatePublisher::odomCallback, this);
  carla_vehicle_status_sub = nh.subscribe(carla_vehicle_status_topic, 1, &CarlaVehicleStatePublisher::vehicleStatusCallback, this);

  twist_pub = nh.advertise<geometry_msgs::TwistStamped>(twist_topic, 1);
  steering_angle_pub = nh.advertise<std_msgs::Float32>(steering_angle_topic, 1);
}

void CarlaVehicleStatePublisher::odomCallback(const nav_msgs::Odometry in_odom_msg) {
  geometry_msgs::TwistStamped out_twist_msg;

  out_twist_msg.header.frame_id = "base_link";
  out_twist_msg.header.stamp = in_odom_msg.header.stamp;
  out_twist_msg.header.seq = in_odom_msg.header.seq;

  out_twist_msg.twist = getBaseLinkTwist(in_odom_msg.twist.twist);

  twist_pub.publish(out_twist_msg);
}

void CarlaVehicleStatePublisher::vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus vehicle_status_msg) {
  std_msgs::Float32 steering_angle_msg;
  steering_angle_msg.data = -vehicle_status_msg.control.steer * max_steering_angle;

  steering_angle_pub.publish(steering_angle_msg);
}

geometry_msgs::Twist CarlaVehicleStatePublisher::getBaseLinkTwist(geometry_msgs::Twist cog_twist) {
  geometry_msgs::Twist base_link_twist;
  base_link_twist.linear.x = sqrt(pow(cog_twist.linear.x, 2) + pow(cog_twist.linear.y, 2));
  base_link_twist.linear.y = 0;
  base_link_twist.linear.z = 0;
  base_link_twist.angular = cog_twist.angular;

  return base_link_twist;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "carla_vehicle_state_publisher_node");
  CarlaVehicleStatePublisher node;
  ros::spin();
  return 0;
}