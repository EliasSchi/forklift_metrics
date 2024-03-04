#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/cache.h"
#include "message_filters/time_synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;

class OdomErrorCalculatorNode : public rclcpp::Node
{
public:
    OdomErrorCalculatorNode() : Node("odom_error_node")
    {
        // odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "odom", 10, std::bind(&OdomErrorCalculatorNode::odom_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose", 10, std::bind(&OdomErrorCalculatorNode::odom_callback, this, std::placeholders::_1));
        ground_truth_subscriber_.subscribe(this,"odom");
        msg_cache_.connectInput(ground_truth_subscriber_);

        initialized_ = false;  
        odom_y_zero_ = 0;
        odom_gt_x_zero_ = 0;
        odom_gt_y_zero_ = 0;
    }

private:
    void odom_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &odom_msg)
    {
         // Extract pose information from the messages
        auto odom_pose = odom_msg->pose.pose;
        auto ground_truth_msg = msg_cache_.getElemBeforeTime(odom_msg->header.stamp);
        auto ground_truth_pose = ground_truth_msg->pose.pose;

        if (initialized_ == false)
        {   
            odom_x_zero_ = odom_pose.position.x;
            odom_y_zero_ = odom_pose.position.y;
            odom_gt_x_zero_ = ground_truth_pose.position.x;
            odom_gt_y_zero_ = ground_truth_pose.position.y;
            initialized_ = true;
        }

        // Calculate the error in the planar space
        double x_error = (ground_truth_pose.position.x - odom_gt_x_zero_) - (odom_pose.position.x - odom_x_zero_);
        double y_error = (ground_truth_pose.position.y - odom_gt_y_zero_) - (odom_pose.position.y - odom_y_zero_);
        double error = std::sqrt(x_error * x_error + y_error * y_error);

        // Output the error
        RCLCPP_INFO(this->get_logger(), "Planar Error: %f", error);
    }
    
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_subscriber_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> ground_truth_subscriber_;
    message_filters::Cache<nav_msgs::msg::Odometry> msg_cache_{10};
    bool initialized_;
    double odom_x_zero_;
    double odom_y_zero_;
    double odom_gt_x_zero_;
    double odom_gt_y_zero_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomErrorCalculatorNode>());
    rclcpp::shutdown();

    return 0;
}