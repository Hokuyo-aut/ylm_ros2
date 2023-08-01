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

#include <chrono>

#include "lumotive_ros2/lumotive_cloud_creation.h"

using namespace std::chrono_literals;

namespace lumotive_pointcloud
{

   ////////////////////////////////////////////////////////////////////////
   // LumotiveFrameCreator class implementation
   ////////////////////////////////////////////////////////////////////////

   /** @brief constructor
   */
    LumotiveFrameCreator::LumotiveFrameCreator(const std::string & node_name, const std::string & node_namespace):
    Node(node_name, node_namespace)
    {
        // Declare parameters
        this->declare_parameter("sensor_ip", "");
        this->declare_parameter("sensor_port", 10940);
        this->declare_parameter("device_frame_id", "");
        this->declare_parameter("nb_packets_per_frame", 0);
        this->declare_parameter("organized_cloud", false);
        this->declare_parameter("color_range_max", -1.0);
        this->declare_parameter("colormap_name", ""); 
        this->declare_parameter("range_min", 0.0);
        this->declare_parameter("range_max", 20.0);
        this->declare_parameter("color_by_reflectivity", false);
        this->declare_parameter("min_brightness", 0.2);
        this->declare_parameter("max_brightness", 10000.0);

        // Get some parameters
        int sub_queue_size, port;
        std::string IP, colormap_name;
        device_id_ = 2; // hardcoded to M20

        this->get_parameter("sensor_ip", IP);
        this->get_parameter("sensor_port", port);
        this->get_parameter("device_frame_id", tf_frame_);
        // this->get_parameter("nb_packets_per_frame", num_packets_per_frame_);
        this->get_parameter("organized_cloud", organized_cloud_);
        this->get_parameter("color_range_max", color_range_max_);
        this->get_parameter("colormap_name", colormap_name);
        this->get_parameter("range_min", min_range_);
        this->get_parameter("range_max", max_range_);
        this->get_parameter("color_by_reflectivity", reflectivity_flag_);
        this->get_parameter("min_brightness", min_r_);
        this->get_parameter("max_brightness", max_r_);

        // Init attributes
        sub_queue_size = 1;

        // Select colormap
        if (colormap_name == "turbo"){
            colormap_in_use_ = turbo_colormap_flipped;
            colormap_in_use_len_ = &turbo_colormap_flipped_len;
        } else if (colormap_name == "inferno") {
            colormap_in_use_ = inferno_colormap;
            colormap_in_use_len_ = &inferno_colormap_len;
        } else if (colormap_name == "viridis") {
            colormap_in_use_ = viridis_colormap;
            colormap_in_use_len_ = &viridis_colormap_len;
        } else if (colormap_name == "greyscale") {
            colormap_in_use_ = greyscale_colormap;
            colormap_in_use_len_ = &greyscale_colormap_len;
        } else if (colormap_name == "greyscale_flipped") {
            colormap_in_use_ = greyscale_colormap_flipped;
            colormap_in_use_len_ = &greyscale_colormap_flipped_len;
        } else {
            colormap_in_use_ = turbo_colormap_flipped;
            colormap_in_use_len_ = &turbo_colormap_flipped_len;
        }

        // create publisher
        lumotive_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lumotive_ros/pointcloud", sub_queue_size);
        pointcloud_publisher_timer_ = this->create_wall_timer(1ms, std::bind(&LumotiveFrameCreator::pointcloud_frame_ready_callback, this));
        
        // empty message timer
        empty_pointcloud_timer_ = this->create_wall_timer(5s, std::bind(&LumotiveFrameCreator::pointcloud_empty_msg_callback, this)); // this timer has oneshot = True, it needs to be re-engaged
        empty_pointcloud_timer_->cancel();

        // Start network
        start_read(IP, port);
    }

    LumotiveFrameCreator::~LumotiveFrameCreator(void) 
    {
        stop_read();
    }

    /** @brief callback to publish pointcloud2 frames that are ready
    *
    */
    void LumotiveFrameCreator::pointcloud_frame_ready_callback(void)
    {
        static bool is_empty_msg_timer_started = false;
        static bool first_message_sent = false; // no need to clear RVIz window before any message has been sent
        sensor_msgs::msg::PointCloud2 output_msg;
        if (is_frame_ready())
        {
            first_message_sent = true;
            is_empty_msg_timer_started = false;
            empty_pointcloud_timer_->cancel();
            convert_lumotive_frame_to_pointcloud2(&output_msg);
            lumotive_pointcloud_publisher_->publish(output_msg);
            pop_front_frame();
        } 
        else if (!is_empty_msg_timer_started && first_message_sent)
        {
            is_empty_msg_timer_started = true;
            empty_pointcloud_timer_->reset();
        }
    }

    /** @brief publishes empty pointcloud message to clear RViz 
     *
     */
    void LumotiveFrameCreator::pointcloud_empty_msg_callback(void)
    {
        sensor_msgs::msg::PointCloud2 output_msg;
        sensor_msgs::PointCloud2Modifier modifier(output_msg);

        output_msg.header.stamp = clock->now();
        output_msg.header.frame_id = tf_frame_;

        // Empty VGA frame
        output_msg.height = 480;
        output_msg.width = 640;
        output_msg.is_dense = false;
        modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensities", 1, sensor_msgs::msg::PointField::UINT16, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        output_msg.data.resize(output_msg.point_step * output_msg.height * output_msg.width, 0);

        //packet_interpretor_->get_empty_pointcloud_message(&output_msg);
        lumotive_pointcloud_publisher_->publish(output_msg);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "(" << tf_frame_ << ") Cleared RVIz window");
    }

    /** @brief method to copy current packet data into frame
     *
     */
    void LumotiveFrameCreator::convert_lumotive_frame_to_pointcloud2(sensor_msgs::msg::PointCloud2 *msg)
    {
        sensor_msgs::PointCloud2Modifier modifier(*msg);
        modifier.clear();

        // Get front item of each stack
        auto mFrame = get_front_mFrame();
        auto rFrame = get_front_rFrame();
        auto iFrame = get_front_iFrame();

        if (!mFrame->range || !mFrame->signal)
        {
            RCLCPP_ERROR(this->get_logger(), "The range and/or signal attribute are not available through the stream but lumotive_ros2 needs it.");
            rclcpp::shutdown();
        }

        if (!organized_cloud_) {
            msg->width = (int) (mFrame->get_image_steer_size() * mFrame->get_image_stare_size());
            msg->height = 1;
            msg->is_dense = true;
            modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensities", 1, sensor_msgs::msg::PointField::UINT16);
            modifier.reserve(msg->width * msg->height);

            sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
            sensor_msgs::PointCloud2Iterator<uint16_t> iter_i(*msg, "intensities");

            uint32_t pixel_idx = 0;
            for (int px = 0; px < (int) msg->width; px++)
            {
                if (rFrame->range[px])
                {
                    *iter_x = rFrame->x[px];
                    *iter_y = rFrame->y[px];
                    *iter_z = rFrame->z[px];
                    *iter_i = iFrame->signal[px];

                    iter_x += 1;
                    iter_y += 1;
                    iter_z += 1;
                    iter_i += 1;
                    pixel_idx += 1;
                }
            }

            modifier.resize(pixel_idx); // Get rid of pixels that aren't valid

        } else {

            msg->width = (int) mFrame->get_image_stare_size();
            msg->height = (int) mFrame->get_image_steer_size();
            msg->is_dense = false;
            modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensities", 1, sensor_msgs::msg::PointField::UINT16, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier.reserve(msg->width * msg->height);

            color RGB_encoding;
            uint32_t computed_offset;

            for (int px = 0; px < (int) (msg->width*msg->height); px++)
            {
                if (rFrame->range[px])
                {
                    uint32_t stare_idx = (uint32_t) px % msg->width;
                    uint32_t steer_idx = (uint32_t) px / msg->width;
                    computed_offset = (uint32_t) (stare_idx + ((msg->height - 1) - steer_idx) * msg->width) * msg->point_step;

                    if (reflectivity_flag_)
                    {
                        encode_color_by_reflectivity_fused_with_range_colormap(&RGB_encoding, iFrame->signal[px], rFrame->range[px]);
                    } else {
                        RGB_encoding = encode_range_to_RGB_with_colormap(rFrame->range[px]);
                    }

                    writePointCloud2BufferValue(&msg->data[computed_offset], sensor_msgs::msg::PointField::FLOAT32, rFrame->x[px]); 
                    writePointCloud2BufferValue(&msg->data[computed_offset+4], sensor_msgs::msg::PointField::FLOAT32, rFrame->y[px]);
                    writePointCloud2BufferValue(&msg->data[computed_offset+8], sensor_msgs::msg::PointField::FLOAT32, rFrame->z[px]);
                    writePointCloud2BufferValue(&msg->data[computed_offset+12], sensor_msgs::msg::PointField::UINT16, iFrame->signal[px]);
                    writePointCloud2BufferValue(&msg->data[computed_offset+14], sensor_msgs::msg::PointField::UINT8, RGB_encoding.b);
                    writePointCloud2BufferValue(&msg->data[computed_offset+15], sensor_msgs::msg::PointField::UINT8, RGB_encoding.g);
                    writePointCloud2BufferValue(&msg->data[computed_offset+16], sensor_msgs::msg::PointField::UINT8, RGB_encoding.r);
                }
            }
        }

        // Fill some common fields
        msg->header.stamp = clock->now();
        msg->header.frame_id = tf_frame_;
    }

    /** @brief method which returns an RGB color value given a range value and the specified colormap
     *  @param range_in_meters range to convert
     *  @return color
     */
    color LumotiveFrameCreator::encode_range_to_RGB_with_colormap(float range_in_meters)
    {   
        color c;

        // Normalize range with respect to range max specified by user
        if (range_in_meters > color_range_max_)
            range_in_meters = range_in_meters - std::floor(range_in_meters / color_range_max_)*color_range_max_;

        // Map onto colormap range
        float color_idx = ( (float) (*colormap_in_use_len_) / color_range_max_) * range_in_meters;
        uint32_t color_idx_rounded = (uint32_t) std::round(color_idx);

        c.r = colormap_in_use_[color_idx_rounded][0];
        c.g = colormap_in_use_[color_idx_rounded][1];
        c.b = colormap_in_use_[color_idx_rounded][2];

        return c;
    }

    /** @brief method which returns an RGB color value given a range value at its input.
     *  @param range_in_meters range to convert
     *  @return color
     */
    color LumotiveFrameCreator::encode_range_to_RGB(float range_in_meters)
    {
        // Interesting read: http://paulbourke.net/miscellaneous/colorspace/
        color c = {255,255,255}; // white

        if (range_in_meters > color_range_max_)
            range_in_meters = range_in_meters - std::floor(range_in_meters / color_range_max_)*color_range_max_;

        if (range_in_meters < (color_range_max_/6)) {
            c.r = (uint8_t) 0;
            c.g = (uint8_t) 255 * 6 * (range_in_meters) / color_range_max_;
        } else if (range_in_meters < (2*color_range_max_/6)) {
            c.r = (uint8_t) 0;
            c.b = (uint8_t) 255 * (1 + 6 * ((1/6) * color_range_max_ - range_in_meters) / color_range_max_);
        } else if (range_in_meters < (3*color_range_max_/6)) {
            c.b = (uint8_t) 0;
            c.r = (uint8_t) 255 * 6 * (range_in_meters - (2/6) * color_range_max_) / color_range_max_;
        } else if (range_in_meters < (4*color_range_max_/6)) {
            c.b = (uint8_t) 0;
            c.g = (uint8_t) 255 * (1 + 6 * ((3/6) * color_range_max_ - range_in_meters) / color_range_max_);
        } else if (range_in_meters < (5*color_range_max_/6)) {
            c.g = (uint8_t) 0;
            c.b = (uint8_t) 255 * 6 * (range_in_meters - (4/6) * color_range_max_) / color_range_max_;
        } else {
            c.g = (uint8_t) 0;
            c.r = (uint8_t) 255 * (1 + 6 * ((6/6) * color_range_max_ - range_in_meters) / color_range_max_);
        }

       return(c);
    }

    /** @brief method which returns if the input range is within the desired bounds
     *  @param range range to check
     *  @return bool
     */
    bool LumotiveFrameCreator::is_range_valid(float range)
    {
        if (range >= min_range_ && range <= max_range_)
            return true;

        return false;
    }

    /** @brief method which returns an RGB color value based on reflectivity
     *  @param intensity point intensity
     *  @param range point range
     *  @return color
     */
    void LumotiveFrameCreator::encode_color_by_reflectivity_fused_with_range_colormap(color *color_coded_range, float intensity, float range)
    {    
        float reflec = intensity * range * range;

        if (reflec > max_r_)
        {
            reflec = max_r_;
        }

        reflec = reflec /max_r_;

        if (reflec < min_r_)
            reflec = min_r_; // Do not allow values that are too low to pass

        color RGB_range = encode_range_to_RGB_with_colormap(range);
        
        //float pic_blue = 2*255 * (range - 2.5)/5.0-255;
        //float pic_red = -pic_blue;
        //if (pic_blue > 255)
        //    pic_blue = 255;
        //if (pic_blue < 0)
        //    pic_blue = 0;
        //if (pic_red > 255)
        //    pic_red = 255;
        //if (pic_red < 0)
        //    pic_red = 0;
        //float pic_green = 200 - pic_blue - pic_red;
                

        color_coded_range->r = (uint8_t) RGB_range.r*reflec;
        color_coded_range->g = (uint8_t) RGB_range.g*reflec;
        color_coded_range->b = (uint8_t) RGB_range.b*reflec;
    }

    /** @brief method which returns a brightness value based on reflectivity
     *  @param intensity point intensity
     *  @param range point range
     *  @return color
     */
    void LumotiveFrameCreator::encode_reflectivity_to_brightness(color *color_coded_range, float intensity, float range)
    {
        float max_r = 12200;
        float min_r = 200;
        float brightness_min = 0;
        float brigntness_max = 255;

        float reflec = intensity * range * range;

        if (reflec > max_r)
        {
            reflec = max_r;
        } else if (reflec < min_r)
        {
            reflec = min_r;   
        }

        reflec = reflec - min_r;
        reflec = reflec / (max_r - min_r);

        float pt_brightness = (float) (brigntness_max - brightness_min) * reflec + brightness_min;

        color_coded_range->r = (uint8_t) pt_brightness;
        color_coded_range->g = (uint8_t) pt_brightness;
        color_coded_range->b = (uint8_t) pt_brightness;
    }
}
