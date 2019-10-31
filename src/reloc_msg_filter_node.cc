#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ViSlam.h>

bool is_init = false;
bool reloc_signal = false;
geometry_msgs::PoseStamped refreshed_reloc_pose;
ros::Time latest_anchor_tp;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_msg_filter");
  ros::NodeHandle n;
  ros::Publisher reloc_msg_pub =
    n.advertise<geometry_msgs::PoseWithCovarianceStamped>("reloc_camera_pose", 1);
  // TODO(meng): subscribed topic name needs to be verified
  ros::Subscriber sub = n.subscribe("vi_slam", 1, rawRelocMsgCallBack);
  ros::Rate loop_rate(20);

  geometry_msgs::PoseWithCovarianceStamped reloc_amcl_pose;
  reloc_amcl_pose.pose.covariance[0] = 0.5 * 0.5;
  reloc_amcl_pose.pose.covariance[7] = 0.5 * 0.5;
  reloc_amcl_pose.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0);
  while (ros::ok())
  {
    ros::spinOnce();
    
    if (reloc_signal) {
      reloc_amcl_pose.header = refreshed_reloc_pose.header;

      reloc_amcl_pose.pose.pose = refreshed_reloc_pose.pose;

      reloc_msg_pub.publish(reloc_amcl_pose);
    }

    loop_rate.sleep();
  }

  return 0;
}
