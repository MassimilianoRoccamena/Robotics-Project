#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

// Queues -------------------------------------------------------------------

#define ODOM_QUEUE      10

// Names --------------------------------------------------------------------

#define NODE_NAME       "fix_camera_odom"

#define PARENT_FRAME    "odom"
#define CHILD_FRAME     "base_link"

//==========================================================================

ros::Subscriber sub;
ros::Publisher pub;

void onOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    nav_msgs::Odometry msg;
    msg.header.stamp = odom->header.stamp;
    msg.header.frame_id = PARENT_FRAME;
    msg.child_frame_id = CHILD_FRAME;
    msg.twist = odom->twist;
    msg.pose = odom->pose;
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    sub = n.subscribe("/camera/odom/sample", ODOM_QUEUE, onOdom);
    pub = n.advertise<nav_msgs::Odometry>("/camera/odom", ODOM_QUEUE);

    ros::spin();
    return 0;
}