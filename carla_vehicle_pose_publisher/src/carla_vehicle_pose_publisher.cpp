
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CarlaVehiclePosePublisher {
 public:
  CarlaVehiclePosePublisher();

  virtual ~CarlaVehiclePosePublisher() {}

 private:
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  ros::Subscriber odom_sub;

  ros::Publisher pose_front_pub;
  ros::Publisher pose_rear_pub;

  std::string odom_topic;
  std::string vehicle_pose_topic_ns;

  double cog_to_front_dist;
  double cog_to_rear_dist;

  void odomCallback(const nav_msgs::Odometry in_odom_msg);

  geometry_msgs::PoseStamped getAxlePose(geometry_msgs::PoseStamped cog_pose, double dist_to_axle, double vehicle_yaw);
};

CarlaVehiclePosePublisher::CarlaVehiclePosePublisher() : tf_listener(tf_buffer) {
  ros::NodeHandle private_nh("~");

  private_nh.param("odom_topic", odom_topic, std::string("/sensors/odometry"));
  private_nh.param("vehicle_pose_topic_ns", vehicle_pose_topic_ns, std::string("/vehicle_pose/"));

  ROS_ASSERT(private_nh.getParam("cog_to_front_dist", cog_to_front_dist));
  ROS_ASSERT(private_nh.getParam("cog_to_rear_dist", cog_to_rear_dist));

  odom_sub = nh.subscribe(odom_topic, 1, &CarlaVehiclePosePublisher::odomCallback, this);

  pose_front_pub = nh.advertise<geometry_msgs::PoseStamped>(vehicle_pose_topic_ns + "front", 1);
  pose_rear_pub = nh.advertise<geometry_msgs::PoseStamped>(vehicle_pose_topic_ns + "rear", 1);
}

void CarlaVehiclePosePublisher::odomCallback(const nav_msgs::Odometry in_odom_msg) {
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

  ROS_INFO_STREAM("POSE_COG_ODOM: " << pose_cog_odom.pose.position.x << " " << pose_cog_odom.pose.position.y);
  ROS_INFO_STREAM("POSE_COG_MAP: " << pose_cog_map.pose.position.x << " " << pose_cog_map.pose.position.y << std::endl);

  double vehicle_yaw;
  double dummy_roll;
  double dummy_pitch;

  tf2::Matrix3x3 mat(quat_tf);
  mat.getEulerYPR(vehicle_yaw, dummy_pitch, dummy_roll);

  geometry_msgs::PoseStamped pose_front_map = getAxlePose(pose_cog_map, cog_to_front_dist, vehicle_yaw);
  geometry_msgs::PoseStamped pose_rear_map = getAxlePose(pose_cog_map, cog_to_rear_dist, vehicle_yaw);

  pose_front_pub.publish(pose_front_map);
  pose_rear_pub.publish(pose_rear_map);
}

geometry_msgs::PoseStamped CarlaVehiclePosePublisher::getAxlePose(geometry_msgs::PoseStamped cog_pose, double dist_to_axle, double vehicle_yaw) {
  geometry_msgs::PoseStamped axle_pose;
  axle_pose.header = cog_pose.header;
  axle_pose.pose.orientation = cog_pose.pose.orientation;
  axle_pose.pose.position.x = cog_pose.pose.position.x + dist_to_axle * cos(vehicle_yaw);
  axle_pose.pose.position.y = cog_pose.pose.position.y + dist_to_axle * sin(vehicle_yaw);

  return axle_pose;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "carla_vehicle_pose_publisher_node");
  CarlaVehiclePosePublisher node;
  ros::spin();
  return 0;
}