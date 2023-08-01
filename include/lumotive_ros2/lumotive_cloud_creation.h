// Copyright (C) 2021-2022 Lumotive
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LUMOTIVE_CLOUD_CREATION_H
#define LUMOTIVE_CLOUD_CREATION_H

#include <string>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_field_conversion.hpp>

#include "lumotive_ros2/lumotive_client_interface.h"
#include "lumotive_ros2/lumotive_colors.h"


namespace lumotive_pointcloud {        
    

#define     EMPTY_MESSAGE_TIMER_SECS             5


class LumotiveFrameCreator: public rclcpp::Node
{
public:
    explicit LumotiveFrameCreator(const std::string & node_name="my_node", const std::string & node_namespace="/");
    ~LumotiveFrameCreator(void);

private:
    int device_id_;
    bool frame_mode_;
    bool organized_cloud_;
    bool reflectivity_flag_;
    bool force_compute_new_res_;
    int nb_packets_in_raw_array_;
    float min_range_;
    float max_range_;
    float min_r_;
    float max_r_;
    std::string tf_frame_;
    float color_range_max_;
    const uint8_t (*colormap_in_use_)[3];
    const uint32_t *colormap_in_use_len_;

    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    rclcpp::TimerBase::SharedPtr pointcloud_publisher_timer_;
    rclcpp::TimerBase::SharedPtr empty_pointcloud_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lumotive_pointcloud_publisher_;
    void convert_lumotive_frame_to_pointcloud2(sensor_msgs::msg::PointCloud2 *msg);
    void pointcloud_frame_ready_callback(void);
    void pointcloud_empty_msg_callback(void);
    color encode_range_to_RGB_with_colormap(float range_in_meters);
    color encode_range_to_RGB(float range_in_meters);
    bool is_range_valid(float range);
    void encode_color_by_reflectivity_fused_with_range_colormap(color *color_coded_range, float intensity, float range);
    void encode_reflectivity_to_brightness(color *color_coded_range, float intensity, float range);
};


// TEMPORARY *******************************************************************************************************************
// The following function once existed in point_field_conversion.h by Sebastian Pütz <spuetz@uni-osnabrueck.de>, 2015.
// But it has been deleted in a recent update. Nevertheless, it is useful to this driver and so we will add it here.
/*!
* \Inserts a given value at the given point position interpreted as the datatype
*  specified by the given datatype parameter.
* \param data_ptr    pointer into the point cloud 2 buffer
* \param datatype    sensor_msgs::PointField datatype value
* \param value       the value to insert
* \tparam T          type of the value to insert
*/
template<typename T>
inline void writePointCloud2BufferValue(unsigned char* data_ptr, const unsigned char datatype, T value){
  switch(datatype){
    case sensor_msgs::msg::PointField::INT8:
    *(reinterpret_cast<int8_t*>(data_ptr)) = static_cast<int8_t>(value);
    break;
    case sensor_msgs::msg::PointField::UINT8:
    *(reinterpret_cast<uint8_t*>(data_ptr)) = static_cast<uint8_t>(value);
    break;
    case sensor_msgs::msg::PointField::INT16:
    *(reinterpret_cast<int16_t*>(data_ptr)) = static_cast<int16_t>(value);
    break;
    case sensor_msgs::msg::PointField::UINT16:
    *(reinterpret_cast<uint16_t*>(data_ptr)) = static_cast<uint16_t>(value);
    break;
    case sensor_msgs::msg::PointField::INT32:
    *(reinterpret_cast<int32_t*>(data_ptr)) = static_cast<int32_t>(value);
    break;
    case sensor_msgs::msg::PointField::UINT32:
    *(reinterpret_cast<uint32_t*>(data_ptr)) = static_cast<uint32_t>(value);
    break;
    case sensor_msgs::msg::PointField::FLOAT32:
    *(reinterpret_cast<float*>(data_ptr)) = static_cast<float>(value);
    break;
    case sensor_msgs::msg::PointField::FLOAT64:
    *(reinterpret_cast<double*>(data_ptr)) = static_cast<double>(value);
    break;
}
}
// END TEMPORARY *******************************************************************************************************************


}

#endif
