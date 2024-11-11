#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "time.h"

class InitAmclPosePublisher : public rclcpp::Node
{
public:
    InitAmclPosePublisher() : Node("init_amcl_pose_publisher")
    {
        this->declare_parameter("x", 0.0);
        this->declare_parameter("y", 0.0);
        this->declare_parameter("theta", 0.0);
        this->declare_parameter("cov", 0.5 * 0.5);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Wait for a subscriber to be available
        while (publisher_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for AMCL Initial Pose subscriber");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void send_init_pose()
    {
        // Retrieve parameters
        double x = this->get_parameter("x").as_double();
        double y = this->get_parameter("y").as_double();
        double theta = this->get_parameter("theta").as_double();
        double cov = this->get_parameter("cov").as_double();

        // Create the message
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;

        // Convert theta to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta);

        msg.pose.pose.orientation.w = quat.w();
        msg.pose.pose.orientation.x = quat.x();
        msg.pose.pose.orientation.y = quat.y();
        msg.pose.pose.orientation.z = quat.z();

        // Set covariance
        msg.pose.covariance = {
            cov, 0.0, 0.0, 0.0, 0.0, 0.0,  // Pos X
            0.0, cov, 0.0, 0.0, 0.0, 0.0,  // Pos Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Pos Z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Rot X
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Rot Y
            0.0, 0.0, 0.0, 0.0, 0.0, cov   // Rot Z
        };

        // Publish the initial pose
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto init_amcl_pose_publisher = std::make_shared<InitAmclPosePublisher>();

    // Send the initial pose
    init_amcl_pose_publisher->send_init_pose();

    // Spin the node to keep it alive
    rclcpp::spin(init_amcl_pose_publisher);

    rclcpp::shutdown();
    return 0;
}
