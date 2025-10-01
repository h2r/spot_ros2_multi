#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <small_gicp/pcl/pcl_registration.hpp>


/*
    * This node subscribes to two PointCloud2 topics, synchronizes them,
    * transforms both to map frame, performs ICP registration,
    * and publishes the merged point cloud.
*/

using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor()
    : Node("point_cloud_processor")
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create synchronized subscribers for both point cloud topics
        sub_pc1_.subscribe(this, "/spot/velodyne/points");
        sub_pc2_.subscribe(this, "/spot2/velodyne/points");

        // Create synchronizer with approximate time policy
        sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
            ApproxSyncPolicy(1), sub_pc1_, sub_pc2_);
        sync_->registerCallback(
            std::bind(&PointCloudProcessor::syncCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Publisher for the merged point cloud
        publisher_full_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/spot/merged_points", 10);
        publisher_realigned2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/spot/realigned2_points", 10);
        publisher_pc2_tf_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
            "/multi_spot/pc2_realign_tf", 10);

        RCLCPP_INFO(this->get_logger(), "Point Cloud Processor with ICP initialized");
    }

private:
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg1,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg2)
    {
        // Transform both point clouds to map frame
        sensor_msgs::msg::PointCloud2 transformed_cloud1, transformed_cloud2;

        if (!transformToMap(msg1, transformed_cloud1) ||
            !transformToMap(msg2, transformed_cloud2)) {
            return;
        }

        // Convert ROS messages to PCL point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud1, *pcl_cloud1);
        pcl::fromROSMsg(transformed_cloud2, *pcl_cloud2);

        // Run ICP to refine alignment
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f cloud2_align_tf;
        auto start = std::chrono::high_resolution_clock::now();
        bool icp_success = performICP(pcl_cloud1, pcl_cloud2, aligned_cloud2, cloud2_align_tf);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        if (!icp_success) {
            RCLCPP_WARN(this->get_logger(),
                ("ICP registration failed after " + std::to_string(duration.count()) + " seconds.").c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), ("ICP took " + std::to_string(duration.count()) + " seconds.").c_str());

        // Log the transformation matrix
        RCLCPP_INFO(this->get_logger(), "ICP Transformation:");
        // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f, %f]",
        //             cloud2_align_tf(0,0), cloud2_align_tf(0,1), cloud2_align_tf(0,2), cloud2_align_tf(0,3));
        // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f, %f]",
        //             cloud2_align_tf(1,0), cloud2_align_tf(1,1), cloud2_align_tf(1,2), cloud2_align_tf(1,3));
        // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f, %f]",
        //             cloud2_align_tf(2,0), cloud2_align_tf(2,1), cloud2_align_tf(2,2), cloud2_align_tf(2,3));
        // RCLCPP_INFO(this->get_logger(), "[%f, %f, %f, %f]",
        //             cloud2_align_tf(3,0), cloud2_align_tf(3,1), cloud2_align_tf(3,2), cloud2_align_tf(3,3));

        // Convert to quaternion and translation
        Eigen::Matrix3f rotation = cloud2_align_tf.block<3,3>(0,0);
        Eigen::Quaternionf quat(rotation);
        Eigen::Vector3f translation = cloud2_align_tf.block<3,1>(0,3);

        RCLCPP_INFO(this->get_logger(), "Translation: [x: %f, y: %f, z: %f]",
                    translation.x(), translation.y(), translation.z());
        RCLCPP_INFO(this->get_logger(), "Rotation (quaternion): [x: %f, y: %f, z: %f, w: %f]",
                    quat.x(), quat.y(), quat.z(), quat.w());

        // Merge the point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *merged_cloud = *pcl_cloud1 + *aligned_cloud2;

        // Convert back to ROS message and publish both the merged and realigned second robot cloud
        sensor_msgs::msg::PointCloud2 merged_output_msg;
        sensor_msgs::msg::PointCloud2 realigned2_msg;
        
        pcl::toROSMsg(*merged_cloud, merged_output_msg);
        merged_output_msg.header.frame_id = "map";
        merged_output_msg.header.stamp = msg1->header.stamp;

        pcl::toROSMsg(*aligned_cloud2, realigned2_msg);
        realigned2_msg.header.frame_id = "map";
        realigned2_msg.header.stamp = msg2->header.stamp;

        // Create and publish transform message
        geometry_msgs::msg::TransformStamped pc2_tf_msg;
        pc2_tf_msg.header.stamp = msg1->header.stamp;
        pc2_tf_msg.header.frame_id = "spot_aligned";
        pc2_tf_msg.child_frame_id = "spot2_aligned";
        pc2_tf_msg.transform.translation.x = translation.x();
        pc2_tf_msg.transform.translation.y = translation.y();
        pc2_tf_msg.transform.translation.z = translation.z();
        pc2_tf_msg.transform.rotation.x = quat.x();
        pc2_tf_msg.transform.rotation.y = quat.y();
        pc2_tf_msg.transform.rotation.z = quat.z();
        pc2_tf_msg.transform.rotation.w = quat.w();

        publisher_pc2_tf_->publish(pc2_tf_msg);
        publisher_full_->publish(merged_output_msg);
        publisher_realigned2_->publish(realigned2_msg);
    }

    bool filterSpotBody(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
    {
        return false; // TODO


    }

    bool transformToMap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input,
                       sensor_msgs::msg::PointCloud2& output)
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("map", input->header.frame_id,
                                           input->header.stamp,
                                           rclcpp::Duration::from_seconds(0.5));
            tf2::doTransform(*input, output, transform_stamped);
            return true;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(),
                       "Could not transform %s to map: %s",
                       input->header.frame_id.c_str(), ex.what());
            return false;
        }
    }

    bool performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned,
                   Eigen::Matrix4f& transformation)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        // small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> icp;
        // icp.setNumThreads(4);

        // Set ICP parameters
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-8);
        icp.setMaxCorrespondenceDistance(0.5);  // 50cm max correspondence
        icp.setEuclideanFitnessEpsilon(1e-5);
        // icp.setRegistrationType("VGICP");

        // Set source and target
        icp.setInputSource(target);  // Cloud 2 (to be aligned)
        icp.setInputTarget(source);  // Cloud 1 (reference)

        // Perform alignment
        icp.align(*aligned);

        if (icp.hasConverged()) {
            transformation = icp.getFinalTransformation();
            RCLCPP_INFO(this->get_logger(),
                       "ICP converged. Score: %f", icp.getFitnessScore());
            return true;
        }

        return false;
    }

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pc1_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pc2_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_full_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_realigned2_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_pc2_tf_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
