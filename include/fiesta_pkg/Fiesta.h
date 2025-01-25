//
// Created by tommy on 4/25/19.
//
// Edited amkulk@udel.edu 
#ifndef ESDF_TOOLS_INCLUDE_FIESTA_H_
#define ESDF_TOOLS_INCLUDE_FIESTA_H_
#include "fiesta_pkg/ESDFMap.h"
#include "fiesta_pkg/raycast.h"
#include "fiesta_pkg/timing.h"
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <unordered_set>
namespace fiesta {

// sensor_msgs::PointCloud2::ConstPtr
// sensor_msgs::Image::ConstPtr

// geometry_msgs::PoseStamped::ConstPtr
// nav_msgs::Odometry::ConstPtr
// geometry_msgs::TransformStamped::ConstPtr
template<class DepthMsgType, class PoseMsgType>
class Fiesta : public rclcpp::Node {
private:
     Parameters parameters_;
     ESDFMap *esdf_map_;
#ifdef SIGNED_NEEDED
     ESDFMap *inv_esdf_map_;
#endif
     bool new_msg_ = false;
     pcl::PointCloud<pcl::PointXYZ> cloud_;
#ifndef PROBABILISTIC
     sensor_msgs::msg::PointCloud2::ConstPtr sync_pc_;
#endif
     typename rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr slice_pub_, text_pub_;
     typename rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr occupancy_pub_;
     typename rclcpp::Subscription<PoseMsgType>::SharedPtr transform_sub_;
     typename rclcpp::Subscription<DepthMsgType>::SharedPtr depth_sub_;
     typename rclcpp::TimerBase::SharedPtr update_mesh_timer_;
     Eigen::Vector3d sync_pos_, raycast_origin_, cur_pos_;
     Eigen::Quaterniond sync_q_;

     std::queue<std::tuple<rclcpp::Time, Eigen::Vector3d, Eigen::Quaterniond>> transform_queue_;
     std::queue<typename DepthMsgType::SharedPtr> depth_queue_;
     typename DepthMsgType::SharedPtr sync_depth_;

     cv::Mat img_[2];
     Eigen::Matrix4d transform_, last_transform_;
     uint image_cnt_ = 0, esdf_cnt_ = 0, tot_ = 0;
#ifdef HASH_TABLE
     std::unordered_set<int> set_free_, set_occ_;
#else
     std::vector<int> set_free_, set_occ_;
#endif
     void Visualization(ESDFMap *esdf_map, bool global_vis, const std::string &text);
#ifdef PROBABILISTIC
     void RaycastProcess(int i, int part, int tt);
     void RaycastMultithread();
#endif

     double GetInterpolation(const cv::Mat &img, double u, double v);

     void DepthConversion();

     void SynchronizationAndProcess();

     void DepthCallback(const typename DepthMsgType::SharedPtr depth_map);

     void PoseCallback(const typename PoseMsgType::SharedPtr msg);

     void UpdateEsdfEvent();

public:
     Fiesta();
     ~Fiesta();
};;

template<class DepthMsgType, class PoseMsgType>
Fiesta<DepthMsgType, PoseMsgType>::Fiesta() : Node("fiesta_node") {
     parameters_.SetParameters(this);
     RCLCPP_INFO(this->get_logger(), "Parameters Set");
#ifdef HASH_TABLE
     esdf_map_ = new ESDFMap(Eigen::Vector3d(0, 0, 0), parameters_.resolution_, parameters_.reserved_size_);
#ifdef SIGNED_NEEDED
       inv_esdf_map_ = new ESDFMap(Eigen::Vector3d(0, 0, 0), parameters_.resolution_, parameters_.reserved_size_);
#endif
#else
     esdf_map_ = new ESDFMap(parameters_.l_cornor_, parameters_.resolution_, parameters_.map_size_);
#ifdef SIGNED_NEEDED
     inv_esdf_map_ = new ESDFMap(parameters_.l_cornor_, parameters_.resolution_, parameters_.map_size_);
#endif
#endif

#ifdef PROBABILISTIC
     esdf_map_->SetParameters(parameters_.p_hit_, parameters_.p_miss_,
                              parameters_.p_min_, parameters_.p_max_, parameters_.p_occ_);
#endif
#ifndef HASH_TABLE
     set_free_.resize(esdf_map_->grid_total_size_);
     set_occ_.resize(esdf_map_->grid_total_size_);
     std::fill(set_free_.begin(), set_free_.end(), 0);
     std::fill(set_occ_.begin(), set_occ_.end(), 0);
#endif
     transform_sub_ = this->create_subscription<PoseMsgType>(
         "transform", 10, std::bind(&Fiesta::PoseCallback, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<DepthMsgType>(
         "depth", 10, std::bind(&Fiesta::DepthCallback, this, std::placeholders::_1));

     slice_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ESDFMap/slice", 1);
     occupancy_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("ESDFMap/occ_pc", 1);
     text_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ESDFMap/text", 1);

     update_mesh_timer_ = this->create_wall_timer(
         std::chrono::duration<double>(parameters_.update_esdf_every_n_sec_),
         std::bind(&Fiesta::UpdateEsdfEvent, this));
     
     RCLCPP_INFO(this->get_logger(), "Fiesta Node Initialized");
}

template<class DepthMsgType, class PoseMsgType>
Fiesta<DepthMsgType, PoseMsgType>::~Fiesta() {
     delete esdf_map_;
#ifdef SIGNED_NEEDED
     delete inv_esdf_map_;
#endif
}

template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::Visualization(ESDFMap *esdf_map, bool global_vis, const std::string &text) {
     if (esdf_map!=nullptr) {
          std::cout << "Visualization" << std::endl;
          if (global_vis)
               esdf_map->SetOriginalRange();
          else
               esdf_map->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_, false);

          sensor_msgs::msg::PointCloud pc;
          esdf_map->GetPointCloud(pc, parameters_.vis_lower_bound_, parameters_.vis_upper_bound_);
          occupancy_pub_->publish(pc);

          visualization_msgs::msg::Marker slice_marker;
          esdf_map->GetSliceMarker(slice_marker, parameters_.slice_vis_level_, 100,
                                   Eigen::Vector4d(0, 1.0, 0, 1), parameters_.slice_vis_max_dist_);
          slice_pub_->publish(slice_marker);
     }
     if (!text.empty()) {
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = "world";
          marker.header.stamp = this->now();
          marker.id = 3456;
          marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          marker.action = visualization_msgs::msg::Marker::MODIFY;

          marker.pose.position.x = 8.0;
          marker.pose.position.y = 2.0;
          marker.pose.position.z = 3.0;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          marker.text = text;

          marker.scale.x = 0.3;
          marker.scale.y = 0.3;
          marker.scale.z = 0.6;

          marker.color.r = 0.0f;
          marker.color.g = 0.0f;
          marker.color.b = 1.0f;
          marker.color.a = 1.0f;
          text_pub_->publish(marker);
     }
}

#ifdef PROBABILISTIC

template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::RaycastProcess(int i, int part, int tt) {
     Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
     for (int idx = part*i; idx < part*(i + 1); idx++) {
          std::vector<Eigen::Vector3d> output;
          if (idx > cloud_.points.size())
               break;
          pcl::PointXYZ pt = cloud_.points[idx];
          int cnt = 0;
          if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
               continue;
          Eigen::Vector4d tmp = transform_*Eigen::Vector4d(pt.x, pt.y, pt.z, 1);
          Eigen::Vector3d point = Eigen::Vector3d(tmp(0), tmp(1), tmp(2))/tmp(3);

          int tmp_idx;
          double length = (point - raycast_origin_).norm();
          if (length < parameters_.min_ray_length_)
               continue;
          else if (length > parameters_.max_ray_length_) {
               point = (point - raycast_origin_)/length*parameters_.max_ray_length_ + raycast_origin_;
               tmp_idx = esdf_map_->SetOccupancy((Eigen::Vector3d) point, 0);
          } else
               tmp_idx = esdf_map_->SetOccupancy((Eigen::Vector3d) point, 1);
#ifdef SIGNED_NEEDED
          tmp_idx = inv_esdf_map_->SetOccupancy((Eigen::Vector3d) point, 0);
#endif
//         //TODO: -10000 ?

          if (tmp_idx!=-10000) {
#ifdef HASH_TABLE
               if (set_occ_.find(tmp_idx) != set_occ_.end())
                   continue;
                 else set_occ_.insert(tmp_idx);
#else
               if (set_occ_[tmp_idx]==tt)
                    continue;
               else
                    set_occ_[tmp_idx] = tt;
#endif
          }
          Raycast(raycast_origin_/parameters_.resolution_,
                  point/parameters_.resolution_,
                  parameters_.l_cornor_/parameters_.resolution_,
                  parameters_.r_cornor_/parameters_.resolution_,
                  &output);

          for (int i = output.size() - 2; i >= 0; i--) {
               Eigen::Vector3d tmp = (output[i] + half)*parameters_.resolution_;

               length = (tmp - raycast_origin_).norm();
               if (length < parameters_.min_ray_length_)
                    break;
               if (length > parameters_.max_ray_length_)
                    continue;
               int tmp_idx;
               tmp_idx = esdf_map_->SetOccupancy(tmp, 0);
#ifdef SIGNED_NEEDED
               tmp_idx = inv_esdf_map_->SetOccupancy(tmp, 1);
#endif
               //TODO: -10000 ?
               if (tmp_idx!=-10000) {
#ifdef HASH_TABLE
                    if (set_free_.find(tmp_idx) != set_free_.end()) {
                        if (++cnt >= 1) {
                          cnt = 0;
                          break;
                        }
                      } else {
                        set_free_.insert(tmp_idx);
                        cnt = 0;
                      }
#else
                    if (set_free_[tmp_idx]==tt) {
                         if (++cnt >= 1) {
                              cnt = 0;
                              break;
                         }
                    } else {
                         set_free_[tmp_idx] = tt;
                         cnt = 0;
                    }
#endif
               }
          }
     }
}

template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::RaycastMultithread() {
     // TODO: when using vector, this is not needed
#ifdef HASH_TABLE
     set_free_.clear();
     set_occ_.clear();
#endif
     int tt = ++tot_;
     timing::Timer raycastingTimer("raycasting");

     if (parameters_.ray_cast_num_thread_==0) {
          RaycastProcess(0, cloud_.points.size(), tt);
     } else {
          int part = cloud_.points.size()/parameters_.ray_cast_num_thread_;
          std::list<std::thread> integration_threads = std::list<std::thread>();
          for (size_t i = 0; i < parameters_.ray_cast_num_thread_; ++i) {
               integration_threads.emplace_back(&Fiesta::RaycastProcess, this, i, part, tt);
          }
          for (std::thread &thread : integration_threads) {
               thread.join();
          }
     }
     raycastingTimer.Stop();
}

#endif // PROBABILISTIC

template<class DepthMsgType, class PoseMsgType>
double Fiesta<DepthMsgType, PoseMsgType>::GetInterpolation(const cv::Mat &img, double u, double v) {
     int vu = img.at<uint16_t>(v, u);
     int v1u = img.at<uint16_t>(v + 1, u);
     int vu1 = img.at<uint16_t>(v, u + 1);
     int v1u1 = img.at<uint16_t>(v + 1, u + 1);
     float a = u - (float) u;
     float c = v - (float) v;
     return (vu*(1.f - a) + vu1*a)*(1.f - c) + (v1u*(1.f - a) + v1u1*a)*c;
}

template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::DepthConversion() {
     timing::Timer depth_timer("depth");
     ++image_cnt_;
     cv::Mat &current_img = img_[image_cnt_ & 1];
     cv::Mat &last_img = img_[!(image_cnt_ & 1)];

     cv_bridge::CvImagePtr cv_ptr;
     cv_ptr = cv_bridge::toCvCopy(depth_queue_.front(), depth_queue_.front()->encoding);
     // TODO: make it a parameter
     constexpr double k_depth_scaling_factor = 1000.0;
     if (depth_queue_.front()->encoding==sensor_msgs::image_encodings::TYPE_32FC1) {
          (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor);
     }
     cv_ptr->image.copyTo(current_img);

     double depth;
     cloud_.clear();
     double uu, vv;

     uint16_t *row_ptr;
     int cols = current_img.cols, rows = current_img.rows;
     if (!parameters_.use_depth_filter_) {
          for (int v = 0; v < rows; v++) {
               row_ptr = current_img.ptr<uint16_t>(v);
               for (int u = 0; u < cols; u++) {
                    depth = (*row_ptr++)/k_depth_scaling_factor;
                    pcl::PointXYZ point;
                    point.x = (u - parameters_.center_x_)*depth/parameters_.focal_x_;
                    point.y = (v - parameters_.center_y_)*depth/parameters_.focal_y_;
                    point.z = depth;
                    cloud_.push_back(point);
               }
          }
     } else {
          if (image_cnt_!=1) {
               Eigen::Vector4d coord_h;
               Eigen::Vector3d coord;
               for (int v = parameters_.depth_filter_margin_; v < rows - parameters_.depth_filter_margin_; v++) {
                    row_ptr = current_img.ptr<uint16_t>(v) + parameters_.depth_filter_margin_;
                    for (int u = parameters_.depth_filter_margin_; u < cols - parameters_.depth_filter_margin_; u++) {
                         depth = (*row_ptr++)/k_depth_scaling_factor;
                         pcl::PointXYZ point;
                         point.x = (u - parameters_.center_x_)*depth/parameters_.focal_x_;
                         point.y = (v - parameters_.center_y_)*depth/parameters_.focal_y_;
                         point.z = depth;
                         if (depth > parameters_.depth_filter_max_dist_ || depth < parameters_.depth_filter_min_dist_)
                              continue;
                         coord_h = last_transform_.inverse()*transform_*Eigen::Vector4d(point.x, point.y, point.z, 1);
                         coord = Eigen::Vector3d(coord_h(0), coord_h(1), coord_h(2))/coord_h(3);
                         uu = coord.x()*parameters_.focal_x_/coord.z() + parameters_.center_x_;
                         vv = coord.y()*parameters_.focal_y_/coord.z() + parameters_.center_y_;
                         if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
//                        getInterpolation(last_img, uu, vv)
                              if (fabs(last_img.at<uint16_t>((int) vv, (int) uu)/k_depth_scaling_factor - coord.z())
                                  < parameters_.depth_filter_tolerance_) {
                                   cloud_.push_back(point);
                              }
                         } //else cloud_.push_back(point_);
                    }
               }
          }
     }
     depth_timer.Stop();
}

template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::SynchronizationAndProcess() {
     rclcpp::Time depth_time;
     double time_delay = 3e-3;
     while (!depth_queue_.empty()) {
          bool new_pos = false;
          depth_time = depth_queue_.front()->header.stamp;
          while (transform_queue_.size() > 1 &&
              std::get<0>(transform_queue_.front()) <= depth_time + rclcpp::Duration::from_seconds(time_delay)) {
               sync_pos_ = std::get<1>(transform_queue_.front());
               sync_q_ = std::get<2>(transform_queue_.front());
               transform_queue_.pop();
               new_pos = true;
          }
          if (transform_queue_.empty()
              || std::get<0>(transform_queue_.front()) <= depth_time + rclcpp::Duration::from_seconds(time_delay)) {
               break;
          }
          if (!new_pos) {
               depth_queue_.pop();
               continue;
          }

          new_msg_ = true;
#ifndef PROBABILISTIC
          // TODO: sync_depth_ must be PointCloud2
            sync_depth_ = depth_queue_.front();
            return;
#else
          if (parameters_.use_depth_filter_)
               last_transform_ = transform_;
          transform_.block<3, 3>(0, 0) = sync_q_.toRotationMatrix();
          transform_.block<3, 1>(0, 3) = sync_pos_;
          transform_(3, 0) = transform_(3, 1) = transform_(3, 2) = 0;
          transform_(3, 3) = 1;
          transform_ = transform_*parameters_.T_D_B_*parameters_.T_B_C_;
          raycast_origin_ = Eigen::Vector3d(transform_(0, 3), transform_(1, 3), transform_(2, 3))/transform_(3, 3);

          if constexpr(std::is_same<DepthMsgType, sensor_msgs::msg::Image>::value) {
               DepthConversion();
          } else if constexpr(std::is_same<DepthMsgType, sensor_msgs::msg::PointCloud2>::value) {
               sensor_msgs::msg::PointCloud2::SharedPtr tmp = depth_queue_.front();
               pcl::fromROSMsg(*tmp, cloud_);
          }

          std::cout << "Pointcloud Size:\t" << cloud_.points.size() << std::endl;
          if ((int) cloud_.points.size()==0) {
               depth_queue_.pop();
               continue;
          }

          RaycastMultithread();
          depth_queue_.pop();
#endif
     }
}

template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::PoseCallback(const typename PoseMsgType::SharedPtr msg) {
     RCLCPP_INFO(this->get_logger(), "Pose Callback");
     Eigen::Vector3d pos;
     Eigen::Quaterniond q;

     if constexpr(std::is_same<PoseMsgType, geometry_msgs::msg::PoseStamped>::value) {
          pos = Eigen::Vector3d(msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z);
          q = Eigen::Quaterniond(msg->pose.orientation.w,
                                 msg->pose.orientation.x,
                                 msg->pose.orientation.y,
                                 msg->pose.orientation.z);
     } else if constexpr(std::is_same<PoseMsgType, nav_msgs::msg::Odometry>::value) {
          pos = Eigen::Vector3d(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
          q = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                 msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z);
     } else if constexpr(std::is_same<PoseMsgType, geometry_msgs::msg::TransformStamped>::value) {
          pos = Eigen::Vector3d(msg->transform.translation.x,
                                msg->transform.translation.y,
                                msg->transform.translation.z);
          q = Eigen::Quaterniond(msg->transform.rotation.w,
                                 msg->transform.rotation.x,
                                 msg->transform.rotation.y,
                                 msg->transform.rotation.z);
     }

     transform_queue_.push(std::make_tuple(msg->header.stamp, pos, q));
}

template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::DepthCallback(const typename DepthMsgType::SharedPtr depth_map) {
     RCLCPP_INFO(this->get_logger(), "Depth Callback");
     depth_queue_.push(depth_map);
     SynchronizationAndProcess();
}
template<class DepthMsgType, class PoseMsgType>
void Fiesta<DepthMsgType, PoseMsgType>::UpdateEsdfEvent() {
     if (!new_msg_)
          return;
     new_msg_ = false;
     cur_pos_ = sync_pos_;

#ifndef PROBABILISTIC
     timing::Timer handlePCTimer("handlePointCloud");
     pcl::fromROSMsg(*sync_pc_, cloud_);

     esdf_map_->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_, false);
     esdf_map_->SetAway();

     Eigen::Vector3i tmp_vox;
     Eigen::Vector3d tmp_pos;
     for (int i = 0; i < cloud_.size(); i++) {
         tmp_pos = Eigen::Vector3d(cloud_[i].x, cloud_[i].y, cloud_[i].z);
         esdf_map_->SetOccupancy(tmp_pos, 1);
     }
     esdf_map_->SetBack();
     handlePCTimer.Stop();
#endif
     esdf_cnt_++;
     std::cout << "Running " << esdf_cnt_ << " updates." << std::endl;

     if (esdf_map_->CheckUpdate()) {
          timing::Timer update_esdf_timer("UpdateESDF");
          if (parameters_.global_update_)
               esdf_map_->SetOriginalRange();
          else
               esdf_map_->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_);
          esdf_map_->UpdateOccupancy(parameters_.global_update_);
          esdf_map_->UpdateESDF();
#ifdef SIGNED_NEEDED
          inv_esdf_map_->UpdateOccupancy();
          inv_esdf_map_->UpdateESDF();
#endif
          update_esdf_timer.Stop();
          timing::Timing::Print(std::cout);
     }

     if (parameters_.visualize_every_n_updates_ != 0 && esdf_cnt_ % parameters_.visualize_every_n_updates_ == 0) {
          Visualization(esdf_map_, parameters_.global_vis_, "");
     }
}
}
#endif //ESDF_TOOLS_INCLUDE_FIESTA_H_
