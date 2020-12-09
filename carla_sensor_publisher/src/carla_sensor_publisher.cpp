#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>

#include "nlohmann/json.hpp"

class CarlaSensorPublisher {
 public:
  CarlaSensorPublisher();

  virtual ~CarlaSensorPublisher() {}

 private:
  ros::NodeHandle nh;

  std::vector<ros::Subscriber> subscribers;
  std::vector<ros::Publisher> publishers;

  std::string sensor_ns;
  std::string sensor_definition_file;

  ros::Subscriber odom_sub;
  ros::Publisher odom_pub;

  std::string odom_src_topic;
  std::string odom_out_topic;
  std::string odom_output_frame;
  std::string odom_output_child_frame;

  void odomCallback(const nav_msgs::Odometry in_odom_msg);

  void parseSensorDefinition();

  template <typename S, typename T>
  void initPubSub(std::string in_topic, std::string out_topic, int sensor_index, std::string sensor_id);

  template <typename S, typename T>
  void sensorMsgCallback(const S in_msg, int sensor_index, std::string frame_id);
};

CarlaSensorPublisher::CarlaSensorPublisher() {
  ros::NodeHandle private_nh("~");

  private_nh.param("sensor_definition_file", sensor_definition_file, std::string("/home/ashuh/carla_ws/src/carla_agv/config/sensors.json"));
  private_nh.param("odom_src_topic", odom_src_topic, std::string("/carla/ego_vehicle/odometry"));
  private_nh.param("odom_out_topic", odom_out_topic, std::string("/odometry"));
  private_nh.param("odom_output_frame", odom_output_frame, std::string("odom"));
  private_nh.param("odom_output_child_frame", odom_output_child_frame, std::string("base_link"));
  private_nh.param("sensor_ns", sensor_ns, std::string("sensors/"));

  parseSensorDefinition();

  odom_sub = nh.subscribe(odom_src_topic, 1, &CarlaSensorPublisher::odomCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>(sensor_ns + odom_out_topic, 1);
}

template <typename S, typename T>
void CarlaSensorPublisher::sensorMsgCallback(const S in_msg, int sensor_index, std::string frame_id) {
  T out_msg;
  out_msg = *in_msg;
  out_msg.header.frame_id = frame_id;
  publishers[sensor_index].publish(out_msg);
}

template <typename S, typename T>
void CarlaSensorPublisher::initPubSub(std::string in_topic, std::string out_topic, int sensor_index, std::string sensor_id) {
  ros::Subscriber sub = nh.subscribe<T>(in_topic, 1, boost::bind(&CarlaSensorPublisher::sensorMsgCallback<S, T>, this, _1, sensor_index, sensor_id));
  subscribers.push_back(sub);

  ros::Publisher pub = nh.advertise<T>(out_topic, 1);
  publishers.push_back(pub);
}

void CarlaSensorPublisher::odomCallback(const nav_msgs::Odometry in_odom_msg) {
  nav_msgs::Odometry out_odom_msg;
  out_odom_msg.header = in_odom_msg.header;
  out_odom_msg.twist.twist.linear.x = in_odom_msg.twist.twist.linear.x;
  out_odom_msg.header.frame_id = odom_output_frame;
  out_odom_msg.child_frame_id = odom_output_child_frame;

  odom_pub.publish(out_odom_msg);
}

void CarlaSensorPublisher::parseSensorDefinition() {
  std::ifstream ifs(sensor_definition_file);
  nlohmann::json j;

  ifs >> j;
  ifs.close();

  int sensor_index = 0;

  const std::string carla_ns = "/carla/ego_vehicle/";

  for (auto &array : j["sensors"]) {
    std::string sensor_type = array["type"];
    std::string sensor_id = array["id"];
    std::string in_topic;
    std::string out_topic;

    sensor_type.erase(remove(sensor_type.begin(), sensor_type.end(), '\"'), sensor_type.end());
    sensor_id.erase(remove(sensor_id.begin(), sensor_id.end(), '\"'), sensor_id.end());

    if (sensor_type == "sensor.camera.rgb" && sensor_id != "view") {
      in_topic = carla_ns + "camera/rgb/" + sensor_id + "/image_color";
      out_topic = sensor_ns + "camera/rgb/" + sensor_id + "/image_color";
      initPubSub<sensor_msgs::Image::ConstPtr, sensor_msgs::Image>(in_topic, out_topic, sensor_index++, sensor_id);
    } else if (sensor_type == "sensor.lidar.ray_cast") {
      in_topic = carla_ns + "lidar/" + sensor_id + "/point_cloud";
      out_topic = sensor_ns + "lidar/" + sensor_id + "/point_cloud";
      initPubSub<sensor_msgs::PointCloud2::ConstPtr, sensor_msgs::PointCloud2>(in_topic, out_topic, sensor_index++, sensor_id);
    } else if (sensor_type == "sensor.other.imu") {
      in_topic = carla_ns + "imu/" + sensor_id;
      out_topic = sensor_ns + "imu/" + sensor_id;
      initPubSub<sensor_msgs::Imu::ConstPtr, sensor_msgs::Imu>(in_topic, out_topic, sensor_index++, sensor_id);
    }
  }

  ROS_INFO("Done parsing sensor definition file");
  ROS_INFO_STREAM("Number of sensors: " << sensor_index);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "carla_sensor_publisher_node");
  CarlaSensorPublisher node;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}