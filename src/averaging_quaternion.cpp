#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Eigenvalues>

geometry_msgs::Vector3 averagingPosition(std::vector<geometry_msgs::TransformStamped> transform_vector)
{
    // Get the number of TF
    const int TF_size = transform_vector.size();

    geometry_msgs::Vector3 position;
    for (const auto& transform : transform_vector)
    {
        // calculate center position
        position.x += transform.transform.translation.x / TF_size;
        position.y += transform.transform.translation.y / TF_size;
        position.z += transform.transform.translation.z / TF_size;
    }

    return position;
}

geometry_msgs::Quaternion averagingQuaternion(std::vector<geometry_msgs::TransformStamped> transform_vector)
{
    // Get the number of TF
    const int TF_size = transform_vector.size();

    // Make M matrix
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(4,4);
    for (const auto& transform : transform_vector)
    {
        tf::Quaternion tf_quat(transform.transform.rotation.x, transform.transform.rotation.y,
                               transform.transform.rotation.z, transform.transform.rotation.w);
        Eigen::Quaterniond eigen_quat;
        tf::quaternionTFToEigen(tf_quat, eigen_quat);
        eigen_quat.normalize();
        Eigen::Vector4d eigen_quat_vec(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());
        Eigen::Matrix4d eigen_quat_mat = eigen_quat_vec * eigen_quat_vec.transpose();
        // std::cout << "eigen_quat_vec" << std::endl;
        // std::cout << eigen_quat_vec << std::endl;
        M += eigen_quat_mat;
    }
    // std::cout << "M" << std::endl;
    // std::cout << M << std::endl;

    // Get maximum eigen vector
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(M);
    Eigen::VectorXd max_eigenvector = solver.eigenvectors().col(solver.eigenvectors().row(0).size() - 1);
    Eigen::Quaterniond eigen_quat_ave(max_eigenvector(0), max_eigenvector(1), max_eigenvector(2), max_eigenvector(3));
    // std::cout << "M's eigenvectors" << std::endl;
    // std::cout << solver.eigenvectors() << std::endl;

    // Data type conversion
    tf::Quaternion tf_quat_ave;
    tf::quaternionEigenToTF(eigen_quat_ave, tf_quat_ave);
    geometry_msgs::Quaternion quaternion_ave;
    quaternionTFToMsg(tf_quat_ave, quaternion_ave);
    // std::cout << "quaternion_ave" << std::endl;
    // std::cout << quaternion_ave << std::endl;

    return quaternion_ave;
}

// Node
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    static tf2_ros::TransformBroadcaster br;

    std::vector<geometry_msgs::TransformStamped> transform_vector;

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        geometry_msgs::TransformStamped transform_A, transform_B;
        try
        {
            transform_A = tfBuffer.lookupTransform("world", "TF_A", ros::Time(0), ros::Duration(1.0));
            transform_B = tfBuffer.lookupTransform("world", "TF_B", ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            continue;
        }

        transform_vector.clear();
        // std::cout << transform_A << std::endl;
        transform_vector.push_back(transform_A);
        // std::cout << transform_B << std::endl;
        transform_vector.push_back(transform_B);

        geometry_msgs::TransformStamped transform_ave;
        transform_ave.header.stamp = ros::Time::now();
        transform_ave.header.frame_id = "world";
        transform_ave.child_frame_id = "TF_average";
        transform_ave.transform.translation = averagingPosition(transform_vector);
        transform_ave.transform.rotation = averagingQuaternion(transform_vector);

        br.sendTransform(transform_ave);
    }
    return 0;
}
