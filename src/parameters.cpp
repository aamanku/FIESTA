//
// Created by tommy on 5/2/19.
//
// Edited amkulk@udel.edu 

#include "fiesta_pkg/parameters.h"
void fiesta::Parameters::SetParameters(rclcpp::Node* node) {
  
  node->declare_parameter<double>("resolution", 0.1);
  node->declare_parameter<int>("visualize_every_n_updates", 1);
  node->declare_parameter<double>("min_ray_length", 0.5);
  node->declare_parameter<double>("max_ray_length", 5.0);
  double slice_vis_level_tmp, vis_lower_bound_tmp, vis_upper_bound_tmp;

  node->declare_parameter<double>("slice_vis_max_dist", 2.0);
  node->declare_parameter<double>("slice_vis_level", 5);
  node->declare_parameter<double>("vis_lower_bound", -5);
  node->declare_parameter<double>("vis_upper_bound", 10);
  node->get_parameter("resolution", resolution_);
  node->get_parameter("visualize_every_n_updates", visualize_every_n_updates_);
  node->get_parameter("min_ray_length", min_ray_length_);
  node->get_parameter("max_ray_length", max_ray_length_);
  node->get_parameter("slice_vis_max_dist", slice_vis_max_dist_);
  node->get_parameter("slice_vis_level", slice_vis_level_tmp);
  node->get_parameter("vis_lower_bound", vis_lower_bound_tmp);
  node->get_parameter("vis_upper_bound", vis_upper_bound_tmp);
  slice_vis_level_ = static_cast<int>(slice_vis_level_tmp / resolution_);
  vis_lower_bound_ = static_cast<int>(vis_lower_bound_tmp / resolution_);
  vis_upper_bound_ = static_cast<int>(vis_upper_bound_tmp / resolution_);

  node->declare_parameter<double>("center_x", 322.477357419);
  node->declare_parameter<double>("center_y", 237.076346481);
  node->declare_parameter<double>("focal_x", 384.458089392);
  node->declare_parameter<double>("focal_y", 383.982755697);
  node->declare_parameter<int>("ray_cast_num_thread", 0);
  node->get_parameter("center_x", center_x_);
  node->get_parameter("center_y", center_y_);
  node->get_parameter("focal_x", focal_x_);
  node->get_parameter("focal_y", focal_y_);
  node->get_parameter("ray_cast_num_thread", ray_cast_num_thread_);
  double radius_x, radius_y, radius_z;
  node->declare_parameter<double>("radius_x", 3.0);
  node->declare_parameter<double>("radius_y", 3.0);
  node->declare_parameter<double>("radius_z", 1.5);
  node->get_parameter("radius_x", radius_x);
  node->get_parameter("radius_y", radius_y);
  node->get_parameter("radius_z", radius_z);
  radius_ = Eigen::Vector3d(radius_x, radius_y, radius_z);

  node->declare_parameter<bool>("global_map", true);
  node->declare_parameter<bool>("global_update", true);
  node->declare_parameter<bool>("global_vis", true);
  node->get_parameter("global_map", global_map_);
  node->get_parameter("global_update", global_update_);
  node->get_parameter("global_vis", global_vis_);
  if (!global_map_)
  global_vis_ = global_update_ = false;

  node->declare_parameter<bool>("use_depth_filter", false);
  node->declare_parameter<double>("depth_filter_tolerance", 0.1);
  node->declare_parameter<double>("depth_filter_max_dist", 10.0);
  node->declare_parameter<double>("depth_filter_min_dist", 0.1);
  node->declare_parameter<int>("depth_filter_margin", 0);
  node->get_parameter("use_depth_filter", use_depth_filter_);
  node->get_parameter("depth_filter_tolerance", depth_filter_tolerance_);
  node->get_parameter("depth_filter_max_dist", depth_filter_max_dist_);
  node->get_parameter("depth_filter_min_dist", depth_filter_min_dist_);
  node->get_parameter("depth_filter_margin", depth_filter_margin_);

#ifdef HASH_TABLE
  l_cornor_ << -100.f, -100.f, -100.f;
  r_cornor_ << 100.f, 100.f, 100.f;
  node->declare_parameter<int>("reserved_size", 1000000);
  node->get_parameter("reserved_size", reserved_size_);
#else
  double lx, ly, lz;
  double rx, ry, rz;
  node->declare_parameter<double>("lx", -20.0);
  node->declare_parameter<double>("ly", -20.0);
  node->declare_parameter<double>("lz", -5.0);
  node->declare_parameter<double>("rx", 20.0);
  node->declare_parameter<double>("ry", 20.0);
  node->declare_parameter<double>("rz", 5.0);
  node->get_parameter("lx", lx);
  node->get_parameter("ly", ly);
  node->get_parameter("lz", lz);
  node->get_parameter("rx", rx);
  node->get_parameter("ry", ry);
  node->get_parameter("rz", rz);

  l_cornor_ << lx, ly, lz;
  r_cornor_ << rx, ry, rz;
  map_size_ = r_cornor_ - l_cornor_;
#endif

  node->declare_parameter<double>("update_esdf_every_n_sec", 0.1);
  node->get_parameter("update_esdf_every_n_sec", update_esdf_every_n_sec_);

  T_B_C_ << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
  // T_D_B_ << 0.971048, -0.120915, 0.206023, 0.00114049,
  //      0.15701, 0.973037, -0.168959, 0.0450936,
  //      -0.180038, 0.196415, 0.96385, 0.0430765,
  //      0.0, 0.0, 0.0, 1.0;

  T_D_B_ << T_B_C_;

#ifdef PROBABILISTIC
  node->declare_parameter<double>("p_hit", 0.70);
  node->declare_parameter<double>("p_miss", 0.35);
  node->declare_parameter<double>("p_min", 0.12);
  node->declare_parameter<double>("p_max", 0.97);
  node->declare_parameter<double>("p_occ", 0.80);
  node->get_parameter("p_hit", p_hit_);
  node->get_parameter("p_miss", p_miss_);
  node->get_parameter("p_min", p_min_);
  node->get_parameter("p_max", p_max_);
  node->get_parameter("p_occ", p_occ_);
#endif
}