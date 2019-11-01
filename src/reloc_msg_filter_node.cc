
#include <string>
#include <utility>

// ROS
#include <message/ViSlam.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

// 3rdparty
#include <glog/logging.h>
#include <Eigen/Geometry>

#include <helper/param_internal.h>

bool is_init = false;
bool reloc_signal = false;
geometry_msgs::PoseStamped refreshed_reloc_pose;
ros::Time latest_anchor_tp;
XP::RelocMsgFilterParam reloc_msg_filter_param;

Eigen::Vector3f rosPointfToEigenVector3f(const geometry_msgs::Point& p) {
  return Eigen::Vector3f{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
}
geometry_msgs::Point eigenVector3fToRosPoint(const Eigen::Vector3f& p) {
  geometry_msgs::Point ros_p;
  ros_p.x = p[0];
  ros_p.y = p[1];
  ros_p.z = p[2];
  return ros_p;
}
Eigen::Matrix3f rosQuaternionToEigenRotationMatrix(const geometry_msgs::Quaternion& q) {
  Eigen::Quaterniond eigen_q;
  eigen_q.x() = q.x;
  eigen_q.y() = q.y;
  eigen_q.z() = q.z;
  eigen_q.w() = q.w;
  return eigen_q.normalized().toRotationMatrix().cast<float>();
}
geometry_msgs::Quaternion eigenRotationMatrixToRosQuaternion(const Eigen::Matrix3f& rot_matrix) {
  Eigen::Quaternionf eigen_q(rot_matrix);
  geometry_msgs::Quaternion ros_q;
  ros_q.x = eigen_q.x();
  ros_q.y = eigen_q.y();
  ros_q.z = eigen_q.z();
  ros_q.w = eigen_q.w();
  return ros_q;
}

void initParameters() {
  const char* env_p = std::getenv("MASTER_DIR");
  if (env_p == nullptr) {
    LOG(FATAL) << "You must set MASTER_DIR first";
  }
  std::string alg_param_file = std::string(env_p) + "/config/reloc_msg_filter.yaml";
  std::string reloc_msg_config_file;
  LOG(INFO) << "loading " << reloc_msg_config_file;
  CHECK(reloc_msg_filter_param.LoadFromYaml(reloc_msg_config_file))
  << "Fail to load reloc msg filter parameters from: "
  << reloc_msg_config_file;
}

void refreshRelocPose(XP::ViSlamConstPtr msg) {
  latest_anchor_tp = msg->anchor_stamp;
  refreshed_reloc_pose = msg->pose_stamped;
  reloc_signal = true;
}

void rawRelocMsgCallBack(XP::ViSlamConstPtr msg) {
  if (!is_init) {
    // first reloc pose received
    refreshRelocPose(msg);
    is_init = true;
  } else if (latest_anchor_tp < msg->anchor_stamp) {
    // new reloc pose received
    refreshRelocPose(msg);
  }
}

int main(int argc, char **argv) {
  initParameters();

  ros::init(argc, argv, "ros_msg_filter");
  ros::NodeHandle n;
  ros::Publisher reloc_msg_pub =
    n.advertise<geometry_msgs::PoseWithCovarianceStamped>("reloc_camera_pose", 1);
  // TODO(meng): subscribed topic name needs to be verified
  ros::Subscriber sub = n.subscribe("vi_slam", 1, rawRelocMsgCallBack);

  geometry_msgs::PoseWithCovarianceStamped reloc_amcl_pose;
  reloc_amcl_pose.pose.covariance[0] = 0.5 * 0.5;
  reloc_amcl_pose.pose.covariance[7] = 0.5 * 0.5;
  reloc_amcl_pose.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0);

  const Eigen::Matrix4f Base_to_Cam = reloc_msg_filter_param.Transform.Cam_to_Base.inverse();
  const Eigen::Matrix4f ViSlamW_to_AmclW = reloc_msg_filter_param.Transform.ViSlamW_to_AmclW;

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    
    if (reloc_signal) {
      // Compute Base_to_AmclW.
      Eigen::Matrix4f Cam_to_ViSlamW = Eigen::Matrix4f::Identity();
      Cam_to_ViSlamW.topRightCorner<3, 1>() =
        rosPointfToEigenVector3f(refreshed_reloc_pose.pose.position);
      Cam_to_ViSlamW.topLeftCorner<3, 3>() =
        rosQuaternionToEigenRotationMatrix(refreshed_reloc_pose.pose.orientation);
      Eigen::Matrix4f Base_to_AmclW = Eigen::Matrix4f::Identity();
      Base_to_AmclW = ViSlamW_to_AmclW * Cam_to_ViSlamW * Base_to_Cam;

      reloc_amcl_pose.header = refreshed_reloc_pose.header;
      reloc_amcl_pose.pose.pose.position =
        eigenVector3fToRosPoint(Base_to_AmclW.topRightCorner<3, 1>());
      reloc_amcl_pose.pose.pose.orientation =
        eigenRotationMatrixToRosQuaternion(Base_to_AmclW.topLeftCorner<3, 3>());

      reloc_msg_pub.publish(reloc_amcl_pose);

      // Print reloc amcl pose int position and quaternion formation.
      ROS_INFO_STREAM("reloc_amcl_pose:\nposition(x, y, z): "
                      << reloc_amcl_pose.pose.pose.position.x << " "
                      << reloc_amcl_pose.pose.pose.position.y << " "
                      << reloc_amcl_pose.pose.pose.position.z
                      << "\nquaternion(x, y, z, w): "
                      << reloc_amcl_pose.pose.pose.orientation.x << " "
                      << reloc_amcl_pose.pose.pose.orientation.y << " "
                      << reloc_amcl_pose.pose.pose.orientation.z << " "
                      << reloc_amcl_pose.pose.pose.orientation.w << "\n");
    }

    loop_rate.sleep();
  }

  return 0;
}
