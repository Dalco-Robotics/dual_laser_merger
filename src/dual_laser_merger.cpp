// Copyright 2024 pradyum
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dual_laser_merger/dual_laser_merger.hpp"

using namespace std::chrono_literals;

namespace merger_node
{
MergerNode::MergerNode(const rclcpp::NodeOptions & options)
: Node("dual_laser_merger", options)
{
  declare_param();

  if (target_frame_param.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Target Frame cannot be Empty");
  } else {
    RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_param.c_str());
  }

  merged_scan_pub =
    this->create_publisher<sensor_msgs::msg::LaserScan>(this->get_parameter(
      "merged_scan_topic").as_string(), rclcpp::SensorDataQoS().reliable());
  merged_cloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter(
      "merged_cloud_topic").as_string(), rclcpp::SensorDataQoS().reliable());
  laser_1_sub.subscribe(this, this->get_parameter("laser_1_topic").as_string(),
      rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  laser_2_sub.subscribe(this, this->get_parameter("laser_2_topic").as_string(),
      rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

  tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer, this);
  message_filter =
    std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>>>(
      message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>(input_queue_size_param),
      laser_1_sub, laser_2_sub);
  message_filter->getPolicy()->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(max_interval_duration)); 
  message_filter->getPolicy()->setInterMessageLowerBound(
      0, rclcpp::Duration::from_seconds(scan_period));
  message_filter->getPolicy()->setInterMessageLowerBound(
      1, rclcpp::Duration::from_seconds(scan_period));
  message_filter->setAgePenalty(tolerance_param);
  message_filter->registerCallback(
    std::bind(&MergerNode::sub_callback, this, std::placeholders::_1, std::placeholders::_2));
  tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void MergerNode::declare_param()
{
  this->declare_parameter("laser_1_topic", "laser_1");
  this->declare_parameter("laser_2_topic", "laser_2");
  this->declare_parameter("merged_scan_topic", "merged");
  this->declare_parameter("merged_cloud_topic", "merged_cloud");
  target_frame_param = this->declare_parameter("target_frame", "");
  tolerance_param = this->declare_parameter("tolerance", 0.01);
  input_queue_size_param =
    this->declare_parameter("queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_param = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_param = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_param = this->declare_parameter("angle_min", -M_PI);
  angle_max_param = this->declare_parameter("angle_max", M_PI);
  angle_increment_param = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_param = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_param = this->declare_parameter("range_min", 0.0);
  range_max_param = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_param = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_param = this->declare_parameter("use_inf", true);
  enable_calibration_param = this->declare_parameter("enable_calibration", false);
  laser_1_x_offset = this->declare_parameter("laser_1_x_offset", 0.0);
  laser_1_y_offset = this->declare_parameter("laser_1_y_offset", 0.0);
  laser_1_yaw_offset = this->declare_parameter("laser_1_yaw_offset", 0.0);
  laser_2_x_offset = this->declare_parameter("laser_2_x_offset", 0.0);
  laser_2_y_offset = this->declare_parameter("laser_2_y_offset", 0.0);
  laser_2_yaw_offset = this->declare_parameter("laser_2_yaw_offset", 0.0);
  allowed_radius_param = this->declare_parameter("allowed_radius", 1.0);
  enable_shadow_filter_param = this->declare_parameter("enable_shadow_filter", false);
  enable_average_filter_param = this->declare_parameter("enable_average_filter", false);
  scan_period = this->declare_parameter("scan_period", 0.1);
  max_interval_duration = this->declare_parameter("max_interval_duration", 0.015);
  verbosity = this->declare_parameter("verbosity", false);
  enable_angle_filter_param = this->declare_parameter("enable_angle_filter", false);
  laser_1_mask_min = this->declare_parameter("laser_1_mask_angle_min", 0.0);
  laser_1_mask_max = this->declare_parameter("laser_1_mask_angle_max", 0.0);
  laser_2_mask_min = this->declare_parameter("laser_2_mask_angle_min", 0.0);
  laser_2_mask_max = this->declare_parameter("laser_2_mask_angle_max", 0.0);
}

void MergerNode::refresh_param()
{
  this->get_parameter("tolerance", tolerance_param);
  this->get_parameter("queue_size", input_queue_size_param);
  this->get_parameter("min_height", min_height_param);
  this->get_parameter("max_height", max_height_param);
  this->get_parameter("angle_min", angle_min_param);
  this->get_parameter("angle_max", angle_max_param);
  this->get_parameter("angle_increment", angle_increment_param);
  this->get_parameter("scan_time", scan_time_param);
  this->get_parameter("range_min", range_min_param);
  this->get_parameter("range_max", range_max_param);
  this->get_parameter("inf_epsilon", inf_epsilon_param);
  this->get_parameter("use_inf", use_inf_param);
  this->get_parameter("laser_1_x_offset", laser_1_x_offset);
  this->get_parameter("laser_1_y_offset", laser_1_y_offset);
  this->get_parameter("laser_1_yaw_offset", laser_1_yaw_offset);
  this->get_parameter("laser_2_x_offset", laser_2_x_offset);
  this->get_parameter("laser_2_y_offset", laser_2_y_offset);
  this->get_parameter("laser_2_yaw_offset", laser_2_yaw_offset);
  this->get_parameter("allowed_radius", allowed_radius_param);
  this->get_parameter("enable_shadow_filter", enable_shadow_filter_param);
  this->get_parameter("enable_average_filter", enable_average_filter_param);
  this->get_parameter("scan_period", scan_period);
  this->get_parameter("max_interval_duration", max_interval_duration);
  this->get_parameter("verbosity", verbosity);
  this->get_parameter("enable_angle_filter", enable_angle_filter_param);
  this->get_parameter("laser_1_mask_angle_min", laser_1_mask_min);
  this->get_parameter("laser_1_mask_angle_max", laser_1_mask_max);
  this->get_parameter("laser_2_mask_angle_min", laser_2_mask_min);
  this->get_parameter("laser_2_mask_angle_max", laser_2_mask_max);
}

void MergerNode::sub_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_1_msg,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_2_msg)
{
  if (target_frame_param.empty()) {
    rclcpp::shutdown();
  } else {

    // log lidar scan msg timesync delay
    auto start = this->now();
    if (verbosity) {
      auto delay1 = start - lidar_1_msg->header.stamp;
      auto delay2 = start - lidar_2_msg->header.stamp;
      auto delay = delay1;
      if (delay2 > delay1) {
        delay = delay2;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "TimeSync delay: " << delay.seconds() * 1000.0 - 100<< " [ms]");
    }
    this->get_parameter<bool>("enable_calibration", enable_calibration_param);
    if(enable_calibration_param) {
      refresh_param();
    }

    lidar_1_filtered = *lidar_1_msg;
    lidar_2_filtered = *lidar_2_msg;

    if (enable_angle_filter_param) {
      apply_angle_mask(lidar_1_filtered, laser_1_mask_min, laser_1_mask_max);
      apply_angle_mask(lidar_2_filtered, laser_2_mask_min, laser_2_mask_max);
    }

    if(enable_average_filter_param) {
      lidar_1_avg = lidar_1_filtered;
      lidar_2_avg = lidar_2_filtered;
      for(size_t i = 0; i < lidar_1_filtered.ranges.size(); i++) {
        if(i == 0) {
          lidar_1_avg.ranges[i] = (lidar_1_filtered.ranges[lidar_1_filtered.ranges.size() - 1] +
            lidar_1_filtered.ranges[i] + lidar_1_filtered.ranges[i + 1]) / 3;
        } else if(i == (lidar_1_filtered.ranges.size() - 1)) {
          lidar_1_avg.ranges[i] = (lidar_1_filtered.ranges[i - 1] + lidar_1_filtered.ranges[i] +
            lidar_1_filtered.ranges[0]) / 3;
        } else {
          lidar_1_avg.ranges[i] = (lidar_1_filtered.ranges[i - 1] + lidar_1_filtered.ranges[i] +
            lidar_1_filtered.ranges[i + 1]) / 3;
        }
      }
      for(size_t i = 0; i < lidar_2_filtered.ranges.size(); i++) {
        if(i == 0) {
          lidar_2_avg.ranges[i] = (lidar_2_filtered.ranges[lidar_2_filtered.ranges.size() - 1] +
            lidar_2_filtered.ranges[i] + lidar_2_filtered.ranges[i + 1]) / 3;
        } else if(i == (lidar_2_filtered.ranges.size() - 1)) {
          lidar_2_avg.ranges[i] = (lidar_2_filtered.ranges[i - 1] + lidar_2_filtered.ranges[i] +
            lidar_2_filtered.ranges[0]) / 3;
        } else {
          lidar_2_avg.ranges[i] = (lidar_2_filtered.ranges[i - 1] + lidar_2_filtered.ranges[i] +
            lidar_2_filtered.ranges[i + 1]) / 3;
        }
      }
    }

    if (enable_angle_filter_param) {
      projector.projectLaser(lidar_1_filtered, cloud_in_1);
      projector.projectLaser(lidar_2_filtered, cloud_in_2);
    } else if(enable_average_filter_param) {
      projector.projectLaser(lidar_1_avg, cloud_in_1);
      projector.projectLaser(lidar_2_avg, cloud_in_2);
    } else {
      projector.projectLaser(*lidar_1_msg, cloud_in_1);
      projector.projectLaser(*lidar_2_msg, cloud_in_2);
    }

    if (lidar_1_msg->header.frame_id != target_frame_param) {
      try {
        cloud_in_1 = tf2_buffer->transform(
          cloud_in_1, target_frame_param, tf2::durationFromSec(tolerance_param));
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure, Laser 1: " << ex.what());
        return;
      }
    }

    if (lidar_2_msg->header.frame_id != target_frame_param) {
      try {
        cloud_in_2 = tf2_buffer->transform(
          cloud_in_2, target_frame_param, tf2::durationFromSec(tolerance_param));
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure, Laser 2: " << ex.what());
        return;
      }
    }

    pcl::fromROSMsg(cloud_in_1, pcl_cloud_in_1);
    pcl::fromROSMsg(cloud_in_2, pcl_cloud_in_2);

    if (pcl_cloud_in_1.points.empty() || pcl_cloud_in_2.points.empty()) {
      return;
    }

    pcl_cloud_out = pcl_cloud_in_1;
    pcl_cloud_out += pcl_cloud_in_2;

    if(enable_shadow_filter_param) {
      allowed_radius_scaled = allowed_radius_param / range_max_param;
      kdtree.setInputCloud(pcl_cloud_out.makeShared());

      for (auto & point : pcl_cloud_out.points) {
        dist_from_origin = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
        numNearbyPoints = kdtree.radiusSearch(point, allowed_radius_scaled * dist_from_origin,
            pointIndices, pointDistances);
        numNearbyPoints -= 1;
        if(numNearbyPoints == 0) {
          if(use_inf_param) {
            point.x = std::numeric_limits<double>::infinity();
            point.y = std::numeric_limits<double>::infinity();
          } else {
            point.x = range_max_param + inf_epsilon_param;
            point.y = range_max_param + inf_epsilon_param;
          }
        }
      }
    }

    pcl::toROSMsg(pcl_cloud_out, cloud_out);
    merged_cloud_pub->publish(cloud_out);

    merged.header = cloud_out.header;
    merged.header.frame_id = target_frame_param;

    merged.angle_min = angle_min_param;
    merged.angle_max = angle_max_param;
    merged.angle_increment = angle_increment_param;
    merged.time_increment = 0.0;
    merged.scan_time = scan_time_param;
    merged.range_min = range_min_param;
    merged.range_max = range_max_param;

    ranges_size = std::ceil((merged.angle_max - merged.angle_min) / merged.angle_increment);

    if (use_inf_param) {
      merged.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    } else {
      merged.ranges.assign(ranges_size, merged.range_max + inf_epsilon_param);
    }

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x"),
      iter_y(cloud_out, "y"), iter_z(cloud_out, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (
        std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) ||
        *iter_z > max_height_param || *iter_z < min_height_param)
      {
        continue;
      }

      range = hypot(*iter_x, *iter_y);
      if (range < merged.range_min || range > merged.range_max) {
        continue;
      }

      angle = atan2(*iter_y, *iter_x);
      if (angle < merged.angle_min || angle > merged.angle_max) {
        continue;
      }

      index = (angle - merged.angle_min) / merged.angle_increment;
      if (range < merged.ranges[index]) {
        merged.ranges[index] = range;
      }
    }

    merged_scan_pub->publish(merged);

    // log lidar scans merge time
    if (verbosity) {
      auto compute_time = this->now() - start;
      RCLCPP_INFO_STREAM(this->get_logger(), "PCL merge time: " << compute_time.seconds() * 1000.0 << " [ms]");
    }
  }
}

void MergerNode::apply_angle_mask(
  sensor_msgs::msg::LaserScan & scan, double mask_min, double mask_max)
{
  if (mask_min == mask_max) {
    return;  // no mask configured for this lidar
  }

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

    bool in_mask;
    if (mask_min <= mask_max) {
      in_mask = (beam_angle >= mask_min && beam_angle <= mask_max);
    } else {
      // sector wraps across the +/-pi boundary
      in_mask = (beam_angle >= mask_min || beam_angle <= mask_max);
    }

    if (in_mask) {
      scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      if (i < scan.intensities.size()) {
        scan.intensities[i] = 0.0f;
      }
    }
  }
}

}  // namespace merger_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(merger_node::MergerNode)
