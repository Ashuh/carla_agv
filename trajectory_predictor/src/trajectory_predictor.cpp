
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "trajectory_predictor/bicycle_model.h"

class TrajectoryPredictor {
 public:
  TrajectoryPredictor();

  virtual ~TrajectoryPredictor() {}

 private:
  ros::NodeHandle nh;

  ros::Timer timer;

  ros::Subscriber pose_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber steering_angle_sub;

  ros::Publisher prediction_pub;

  std::string pose_topic;
  std::string odom_topic;
  std::string steering_angle_topic;
  std::string predicted_poses_topic;

  double wheelbase;
  double prediction_timestep;
  double prediction_max_time;

  double current_steering_angle;
  double current_velocity;
  geometry_msgs::Pose current_pose;
  nav_msgs::Odometry current_odometry;

  void poseCallback(const geometry_msgs::PoseStamped pose_msg);
  void odomCallback(const nav_msgs::Odometry odom_msg);
  void steeringAngleCallback(const std_msgs::Float32 steering_angle_msg);
  void timerCallback(const ros::TimerEvent& timer_event);
};

TrajectoryPredictor::TrajectoryPredictor() {
  ros::NodeHandle private_nh("~");

  private_nh.param("pose_topic", pose_topic, std::string("/current_pose"));
  private_nh.param("odom_topic", odom_topic, std::string("/odometry/filtered"));
  private_nh.param("steering_angle_topic", steering_angle_topic, std::string("/current_steering_angle"));
  private_nh.param("predicted_poses_topic", predicted_poses_topic, std::string("/predicted_poses"));

  private_nh.param("prediction_timestep", prediction_timestep, 0.1);
  private_nh.param("prediction_max_time", prediction_max_time, 5.0);

  ROS_ASSERT(private_nh.getParam("wheelbase", wheelbase));

  timer = nh.createTimer(ros::Duration(1.0 / 10.0), &TrajectoryPredictor::timerCallback, this);

  pose_sub = nh.subscribe(pose_topic, 1, &TrajectoryPredictor::poseCallback, this);
  odom_sub = nh.subscribe(odom_topic, 1, &TrajectoryPredictor::odomCallback, this);
  steering_angle_sub = nh.subscribe(steering_angle_topic, 1, &TrajectoryPredictor::steeringAngleCallback, this);

  prediction_pub = nh.advertise<geometry_msgs::PoseArray>(predicted_poses_topic, 1);
}

void TrajectoryPredictor::timerCallback(const ros::TimerEvent& timer_event) {
  double x = current_pose.position.x;
  double y = current_pose.position.y;
  double velocity = current_velocity;
  double theta;

  tf2::Quaternion q;
  tf2::fromMsg(current_pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double dummy;
  m.getEulerYPR(theta, dummy, dummy);

  geometry_msgs::PoseArray predicted_poses;

  velocity = std::max(1.0, velocity);

  BicycleModel model(x, y, theta, current_steering_angle, 0.0, velocity, wheelbase);
  predicted_poses = model.predict(prediction_max_time, prediction_timestep);
  prediction_pub.publish(predicted_poses);
}

void TrajectoryPredictor::poseCallback(const geometry_msgs::PoseStamped pose_msg) {
  current_pose = pose_msg.pose;
}

void TrajectoryPredictor::odomCallback(const nav_msgs::Odometry odom_msg) {
  current_velocity = sqrt(pow(odom_msg.twist.twist.linear.x, 2) + pow(odom_msg.twist.twist.linear.y, 2));
}

void TrajectoryPredictor::steeringAngleCallback(const std_msgs::Float32 steering_angle_msg) {
  current_steering_angle = steering_angle_msg.data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_predictor_node");
  TrajectoryPredictor node;
  ros::spin();
  return 0;
}