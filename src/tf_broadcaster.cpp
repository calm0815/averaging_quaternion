#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

geometry_msgs::TransformStamped setTransform(std::string tf_name,
                                            double x, double y, double z, double roll, 
                                            double pitch, double yaw)
{
    geometry_msgs::TransformStamped transformStamped;

    // set info
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = tf_name;
    
    // set position
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    // set orientation
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    return transformStamped;
}

// Node
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;
    static tf2_ros::TransformBroadcaster br;

    double deg = 0;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        br.sendTransform(setTransform("TF_A", 0.0, 0.5, 0.0, deg, deg, deg));
        br.sendTransform(setTransform("TF_B", 0.0, -0.5, 0.0, deg, deg, deg));
        deg += 0.03;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
