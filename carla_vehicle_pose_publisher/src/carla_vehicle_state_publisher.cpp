
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CarlaVehicleStatePublisher {
 public:
  CarlaVehicleStatePublisher();

  virtual ~CarlaVehicleStatePublisher() {}

 private:
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  ros::Subscriber odom_sub;
  ros::Subscriber carla_vehicle_status_sub;

  ros::Publisher pose_pub;
  ros::Publisher velocity_pub;
  ros::Publisher steering_angle_pub;

  std::string odom_topic;
  std::string carla_vehicle_info_topic;

  std::string vehicle_pose_topic;
  std::string velocity_topic;
  std::string steering_angle_topic;

  double cog_to_rear_dist;

  void odomCallback(const nav_msgs::Odometry in_odom_msg);
  void vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus vehicle_status_msg);
  geometry_msgs::PoseStamped getAxlePose(geometry_msgs::PoseStamped cog_pose, double dist_to_axle, double vehicle_yaw);
};

CarlaVehicleStatePublisher::CarlaVehicleStatePublisher() : tf_listener(tf_buffer) {
  ros::NodeHandle private_nh("~");

  private_nh.param("odom_topic", odom_topic, std::string("/odometry/filtered"));
  private_nh.param("carla_vehicle_info_topic", carla_vehicle_info_topic, std::string("/carla/ego_vehicle/vehicle_status"));

  private_nh.param("vehicle_pose_topic", vehicle_pose_topic, std::string("/current_pose"));
  private_nh.param("velocity_topic", velocity_topic, std::string("/current_velocity"));
  private_nh.param("steering_angle_topic", steering_angle_topic, std::string("/current_steering_angle"));

  ROS_ASSERT(private_nh.getParam("cog_to_rear_dist", cog_to_rear_dist));

  odom_sub = nh.subscribe(odom_topic, 1, &CarlaVehicleStatePublisher::odomCallback, this);
  carla_vehicle_status_sub = nh.subscribe(carla_vehicle_info_topic, 1, &CarlaVehicleStatePublisher::vehicleStatusCallback, this);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>(vehicle_pose_topic, 1);
  velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(velocity_topic, 1);
  steering_angle_pub = nh.advertise<std_msgs::Float32>(steering_angle_topic, 1);
}

void CarlaVehicleStatePublisher::odomCallback(const nav_msgs::Odometry in_odom_msg) {
  geometry_msgs::TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped pose_cog_odom;
  geometry_msgs::PoseStamped pose_cog_map;

  pose_cog_odom.header.frame_id = in_odom_msg.header.frame_id;
  pose_cog_odom.header.stamp = in_odom_msg.header.stamp;
  pose_cog_odom.pose = in_odom_msg.pose.pose;

  tf2::doTransform(pose_cog_odom, pose_cog_map, transform_stamped);

  tf2::Quaternion quat_tf(pose_cog_map.pose.orientation.x, pose_cog_map.pose.orientation.y,
                          pose_cog_map.pose.orientation.z, pose_cog_map.pose.orientation.w);

  double vehicle_yaw;
  double dummy_roll;
  double dummy_pitch;

  tf2::Matrix3x3 mat(quat_tf);
  mat.getEulerYPR(vehicle_yaw, dummy_pitch, dummy_roll);

  geometry_msgs::PoseStamped pose_rear_map = getAxlePose(pose_cog_map, cog_to_rear_dist, vehicle_yaw);
  pose_pub.publish(pose_rear_map);

  geometry_msgs::TwistStamped out_twist_msg;
  out_twist_msg.header = in_odom_msg.header;
  out_twist_msg.header.frame_id = "base_link";
  out_twist_msg.twist = in_odom_msg.twist.twist;

  velocity_pub.publish(out_twist_msg);
}

void CarlaVehicleStatePublisher::vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus vehicle_status_msg) {
  std_msgs::Float32 steering_angle_msg;
  steering_angle_msg.data = vehicle_status_msg.control.steer;

  steering_angle_pub.publish(steering_angle_msg);
}

geometry_msgs::PoseStamped CarlaVehicleStatePublisher::getAxlePose(geometry_msgs::PoseStamped cog_pose, double dist_to_axle, double vehicle_yaw) {
  geometry_msgs::PoseStamped axle_pose;
  axle_pose.header = cog_pose.header;
  axle_pose.pose.orientation = cog_pose.pose.orientation;
  axle_pose.pose.position.x = cog_pose.pose.position.x + dist_to_axle * cos(vehicle_yaw);
  axle_pose.pose.position.y = cog_pose.pose.position.y + dist_to_axle * sin(vehicle_yaw);

  return axle_pose;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "carla_vehicle_pose_publisher_node");
  CarlaVehicleStatePublisher node;
  ros::spin();
  return 0;
}