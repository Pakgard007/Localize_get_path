#include <lidar_localization/lidar_localization_component.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>
#include <functional>
#include <iomanip>

namespace
{
// How much odom history to keep for scan-latency compensation. The window only
// has to cover one scan period plus the registration time; 2 s is generous.
constexpr double kOdomBufferSpanSec = 2.0;

// Odom messages the middleware may hold while a scan is being registered. The
// executor is single-threaded, so nothing is taken during align(); with the
// sensor-data default of 5 everything older was dropped, and after a stall
// longer than the 1 s guard in odomReceived the motion over that interval was
// lost outright (2026-10-02: a 2 s stall left the pose 3.7 m behind the cart).
constexpr size_t kOdomQueueDepth = 1000;

// How far short of the end of a sweep the odom buffer may fall and still be
// used to deskew it. A cloud reaches this node only a few ms after its sweep
// ends, so demanding an odom stamp at or past that instant skipped the deskew
// on most scans. integrateOdom stops at the last sample it has, so the tail
// of the sweep is merely under-corrected by v * shortfall (under 1 cm at
// 200 Hz odom), not extrapolated.
constexpr double kDeskewEndToleranceSec = 0.03;

// Distance at which a scan point counts as "explained by the map" when score
// trimming is off, so the inlier-ratio diagnostic is always meaningful.
constexpr double kDefaultInlierDistanceM = 1.0;

bool isNdtRegistrationMethod(const std::string & registration_method)
{
  return registration_method == "NDT" || registration_method == "NDT_OMP";
}

double computeMinimumSafeNdtResolution(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud)
{
  if (!cloud || cloud->empty()) {
    return 0.0;
  }

  double min_x = cloud->points.front().x;
  double max_x = cloud->points.front().x;
  double min_y = cloud->points.front().y;
  double max_y = cloud->points.front().y;
  double min_z = cloud->points.front().z;
  double max_z = cloud->points.front().z;

  for (const auto & point : cloud->points) {
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
    min_z = std::min(min_z, static_cast<double>(point.z));
    max_z = std::max(max_z, static_cast<double>(point.z));
  }

  constexpr double kMinAxisSpan = 1e-3;
  const double x_span = std::max(max_x - min_x, kMinAxisSpan);
  const double y_span = std::max(max_y - min_y, kMinAxisSpan);
  const double z_span = std::max(max_z - min_z, kMinAxisSpan);
  const double span_volume = x_span * y_span * z_span;

  return std::cbrt(span_volume / static_cast<double>(std::numeric_limits<int>::max()));
}
}  // namespace

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("enable_map_odom_tf", false);
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_exclude_z_min", 0.0);
  declare_parameter("scan_exclude_z_max", 0.0);
  declare_parameter("fitness_max_correspondence_distance", 0.0);
  declare_parameter("min_inlier_ratio", 0.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_scan_latency_compensation", true);
  declare_parameter("use_scan_deskew", true);
  declare_parameter("use_scan_time_init_guess", true);
  declare_parameter("max_latency_compensation_sec", 0.5);
  declare_parameter("max_correction_distance", 0.0);
  declare_parameter("max_correction_reject_streak", 10);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
  declare_parameter("enable_timer_publishing", false);
  declare_parameter("pose_publish_frequency", 10.0);
  declare_parameter("path_max_length", 2000);
  declare_parameter("use_gnss_recovery", false);
  declare_parameter("gnss_topic", "/ublox_gps_node/fix");
  declare_parameter("gnss_origin_lat", 0.0);
  declare_parameter("gnss_origin_lon", 0.0);
  declare_parameter("gnss_to_map_yaw_offset_deg", 0.0);
  declare_parameter("gnss_max_covariance", 1.0);
  declare_parameter("gnss_max_age_sec", 1.0);
  declare_parameter("gnss_lost_scan_threshold", 30);
  declare_parameter("gnss_ndt_disagreement_warn_m", 3.0);
  declare_parameter("correction_accept_inlier_gain", 0.0);
  declare_parameter("probe_inlier_threshold", 0.0);
  declare_parameter("probe_offset_m", 1.5);
  declare_parameter("probe_min_interval_sec", 1.0);
  declare_parameter("probe_accept_inlier_gain", 0.08);
  declare_parameter("debug_csv_path", "");
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();
  if (use_gnss_recovery_ && gnss_ndt_disagreement_pub_) {
    gnss_ndt_disagreement_pub_->on_activate();
  }
  match_inlier_ratio_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;

    if (use_gnss_recovery_ && gnss_fix_available_) {
      RCLCPP_INFO(
        get_logger(),
        "Using gnss fix for initial pose instead of the configured initial_pose_x/y: "
        "x=%.3f, y=%.3f", last_gnss_map_x_, last_gnss_map_y_);
      msg->pose.pose.position.x = last_gnss_map_x_;
      msg->pose.pose.position.y = last_gnss_map_y_;
    } else {
      msg->pose.pose.position.x = initial_pose_x_;
      msg->pose.pose.position.y = initial_pose_y_;
    }
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // load a pcd or ply file
    if (map_path_.rfind(".pcd") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Loading pcd map from: %s", map_path_.c_str());
      if (pcl::io::loadPCDFile(map_path_, *map_cloud_ptr) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load pcd file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else if (map_path_.rfind(".ply") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Loading ply map from: %s", map_path_.c_str());
      if (pcl::io::loadPLYFile(map_path_, *map_cloud_ptr) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load ply file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else {
      RCLCPP_ERROR(
          get_logger(), "Unsupported map file format. Please use .pcd or .ply: %s",
          map_path_.c_str());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    if (isNdtRegistrationMethod(registration_method_)) {
      const double min_safe_resolution = computeMinimumSafeNdtResolution(map_cloud_ptr);
      if (min_safe_resolution > 0.0 && ndt_resolution_ < min_safe_resolution) {
        RCLCPP_ERROR(
          get_logger(),
          "ndt_resolution %.3f m is too small for this map extent and would overflow the "
          "NDT voxel grid. Increase ndt_resolution to at least %.3f m.",
          ndt_resolution_, min_safe_resolution);
        return CallbackReturn::FAILURE;
      }
    }

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();
  if (use_gnss_recovery_ && gnss_ndt_disagreement_pub_) {
    gnss_ndt_disagreement_pub_->on_deactivate();
  }
  match_inlier_ratio_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();
  gnss_sub_.reset();
  gnss_ndt_disagreement_pub_.reset();
  match_inlier_ratio_pub_.reset();

  if (enable_timer_publishing_){
    pose_publish_timer_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("enable_map_odom_tf", enable_map_odom_tf_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_exclude_z_min", scan_exclude_z_min_);
  get_parameter("scan_exclude_z_max", scan_exclude_z_max_);
  get_parameter("fitness_max_correspondence_distance", fitness_max_correspondence_distance_);
  get_parameter("min_inlier_ratio", min_inlier_ratio_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_scan_latency_compensation", use_scan_latency_compensation_);
  get_parameter("use_scan_deskew", use_scan_deskew_);
  get_parameter("use_scan_time_init_guess", use_scan_time_init_guess_);
  get_parameter("max_latency_compensation_sec", max_latency_compensation_sec_);
  get_parameter("max_correction_distance", max_correction_distance_);
  get_parameter("max_correction_reject_streak", max_correction_reject_streak_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);
  get_parameter("enable_timer_publishing", enable_timer_publishing_);
  get_parameter("pose_publish_frequency", pose_publish_frequency_);
  get_parameter("path_max_length", path_max_length_);
  get_parameter("use_gnss_recovery", use_gnss_recovery_);
  get_parameter("gnss_topic", gnss_topic_);
  get_parameter("gnss_origin_lat", gnss_origin_lat_);
  get_parameter("gnss_origin_lon", gnss_origin_lon_);
  double gnss_to_map_yaw_offset_deg;
  get_parameter("gnss_to_map_yaw_offset_deg", gnss_to_map_yaw_offset_deg);
  gnss_to_map_yaw_offset_rad_ = gnss_to_map_yaw_offset_deg * M_PI / 180.0;
  get_parameter("gnss_max_covariance", gnss_max_covariance_);
  get_parameter("gnss_max_age_sec", gnss_max_age_sec_);
  get_parameter("gnss_lost_scan_threshold", gnss_lost_scan_threshold_);
  get_parameter("gnss_ndt_disagreement_warn_m", gnss_ndt_disagreement_warn_m_);
  get_parameter("correction_accept_inlier_gain", correction_accept_inlier_gain_);
  get_parameter("probe_inlier_threshold", probe_inlier_threshold_);
  get_parameter("probe_offset_m", probe_offset_m_);
  get_parameter("probe_min_interval_sec", probe_min_interval_sec_);
  get_parameter("probe_accept_inlier_gain", probe_accept_inlier_gain_);
  get_parameter("debug_csv_path", debug_csv_path_);
  RCLCPP_INFO(get_logger(),"correction_accept_inlier_gain: %lf", correction_accept_inlier_gain_);
  RCLCPP_INFO(get_logger(),"probe_inlier_threshold: %lf", probe_inlier_threshold_);
  RCLCPP_INFO(get_logger(),"probe_offset_m: %lf", probe_offset_m_);
  if (!debug_csv_path_.empty()) {
    debug_csv_.open(debug_csv_path_);
    debug_csv_ << "stamp,age_ms,odom_lead_ms,n_src,deskewed,walked_back,prior_x,prior_y,prior_yaw,"
      "iters,pre_ms,align_ms,eval_ms,total_ms,fitness,inlier,ndt_x,ndt_y,ndt_z,ndt_yaw,"
      "correction,status,pub_x,pub_y,pub_yaw,pub_t,tp,prior_inlier,probe\n";
    RCLCPP_INFO(get_logger(),"debug_csv_path: %s", debug_csv_path_.c_str());
  }

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"enable_map_odom_tf: %d", enable_map_odom_tf_);
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_exclude_z: %lf .. %lf", scan_exclude_z_min_, scan_exclude_z_max_);
  RCLCPP_INFO(
    get_logger(),"fitness_max_correspondence_distance: %lf",
    fitness_max_correspondence_distance_);
  RCLCPP_INFO(get_logger(),"min_inlier_ratio: %lf", min_inlier_ratio_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_scan_latency_compensation: %d", use_scan_latency_compensation_);
  RCLCPP_INFO(get_logger(),"use_scan_deskew: %d", use_scan_deskew_);
  RCLCPP_INFO(get_logger(),"use_scan_time_init_guess: %d", use_scan_time_init_guess_);
  RCLCPP_INFO(get_logger(),"max_latency_compensation_sec: %lf", max_latency_compensation_sec_);
  RCLCPP_INFO(get_logger(),"max_correction_distance: %lf", max_correction_distance_);
  RCLCPP_INFO(get_logger(),"max_correction_reject_streak: %d", max_correction_reject_streak_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(),"enable_timer_publishing: %d", enable_timer_publishing_);
  RCLCPP_INFO(get_logger(),"pose_publish_frequency: %lf", pose_publish_frequency_);
  RCLCPP_INFO(get_logger(),"path_max_length: %d", path_max_length_);
  RCLCPP_INFO(get_logger(),"use_gnss_recovery: %d", use_gnss_recovery_);
  if (use_gnss_recovery_) {
    RCLCPP_INFO(get_logger(),"gnss_topic: %s", gnss_topic_.c_str());
    RCLCPP_INFO(get_logger(),"gnss_origin_lat: %lf", gnss_origin_lat_);
    RCLCPP_INFO(get_logger(),"gnss_origin_lon: %lf", gnss_origin_lon_);
    RCLCPP_INFO(get_logger(),"gnss_to_map_yaw_offset_deg: %lf", gnss_to_map_yaw_offset_deg);
    RCLCPP_INFO(get_logger(),"gnss_max_covariance: %lf", gnss_max_covariance_);
    RCLCPP_INFO(get_logger(),"gnss_lost_scan_threshold: %d", gnss_lost_scan_threshold_);
    RCLCPP_INFO(get_logger(),"gnss_ndt_disagreement_warn_m: %lf", gnss_ndt_disagreement_warn_m_);
  }
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS().keep_last(kOdomQueueDepth),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    // Depth 1: a scan that waited behind another is already stale, and
    // registering it only pushes the newest one further back.
    "cloud", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  if (use_gnss_recovery_) {
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::gnssReceived, this, std::placeholders::_1));

    gnss_ndt_disagreement_pub_ = create_publisher<std_msgs::msg::Float64>(
      "gnss_ndt_disagreement_m",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  }

  // Always published: this is the number that tells you whether a poor match
  // means "the map is out of date / the lot is full of cars" or "I am lost".
  match_inlier_ratio_pub_ = create_publisher<std_msgs::msg::Float64>(
    "match_inlier_ratio",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  if (enable_timer_publishing_) {
    auto period = std::chrono::duration<double>(1.0 / pose_publish_frequency_);
    pose_publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PCLLocalization::timerPublishPose, this));
  }

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  registration_->setMaximumIterations(ndt_max_iterations_);


  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;

  path_ptr_->header.stamp = msg->header.stamp;
  path_ptr_->header.frame_id = global_frame_id_;
  path_ptr_->poses.clear();

  geometry_msgs::msg::PoseStamped initial_pose_stamped;
  initial_pose_stamped.header.stamp = msg->header.stamp;
  initial_pose_stamped.header.frame_id = global_frame_id_;
  initial_pose_stamped.pose = msg->pose.pose;
  path_ptr_->poses.push_back(initial_pose_stamped);

  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
  path_pub_->publish(*path_ptr_);

  if(last_scan_ptr_) {
    cloudReceived(last_scan_ptr_);
  }

  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (isNdtRegistrationMethod(registration_method_)) {
    const double min_safe_resolution = computeMinimumSafeNdtResolution(map_cloud_ptr);
    if (min_safe_resolution > 0.0 && ndt_resolution_ < min_safe_resolution) {
      RCLCPP_ERROR(
        get_logger(),
        "ndt_resolution %.3f m is too small for this map extent and would overflow the "
        "NDT voxel grid. Increase ndt_resolution to at least %.3f m.",
        ndt_resolution_, min_safe_resolution);
      return;
    }
  }

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

PCLLocalization::MatchQuality PCLLocalization::evaluateMatch(
  const pcl::PointCloud<pcl::PointXYZI> & aligned_cloud) const
{
  // Same one-nearest-neighbour sweep that pcl::Registration::getFitnessScore
  // does, over the cloud align() already transformed for us, but it also
  // counts how many scan points found a map point at all.
  //
  // The trimming matters wherever the world has gained things since the map
  // was surveyed. A car parked in a bay that was empty on survey day has no
  // map correspondence, so its nearest map point is the ground or the wall
  // metres away, and the plain fitness score charges that squared distance
  // against the match -- in a full lot 20-25% of the scan can be unmapped,
  // enough to push a perfectly good match over score_threshold and report
  // "lost" while the pose is fine. Excluding those points from the average
  // and reporting them as a ratio separates "the map is stale here" from
  // "the pose is wrong", which the single old number could not do.
  MatchQuality quality;
  if (aligned_cloud.empty()) {
    return quality;
  }

  const auto tree = registration_->getSearchMethodTarget();
  if (!tree) {
    return quality;
  }

  const bool trim_score = fitness_max_correspondence_distance_ > 0.0;
  const double trim_distance_sq = trim_score ?
    fitness_max_correspondence_distance_ * fitness_max_correspondence_distance_ :
    std::numeric_limits<double>::max();
  // The ratio needs a real distance even when the score is left untrimmed,
  // otherwise every point counts as explained and the diagnostic reads 100%
  // no matter what the scan contains.
  const double inlier_distance_sq = trim_score ?
    trim_distance_sq :
    kDefaultInlierDistanceM * kDefaultInlierDistanceM;

  double squared_distance_sum = 0.0;
  long scored = 0;
  long inliers = 0;
  const long point_count = static_cast<long>(aligned_cloud.points.size());

  // One kd-tree lookup per scan point against the full map: ~35 ms on a single
  // thread, a third of the scan period, and the searches are independent.
  const int threads = ndt_num_threads_ > 0 ? ndt_num_threads_ : omp_get_max_threads();
  #pragma omp parallel num_threads(threads) reduction(+:squared_distance_sum, scored, inliers)
  {
    std::vector<int> nn_indices(1);
    std::vector<float> nn_squared_distances(1);
    #pragma omp for schedule(static)
    for (long i = 0; i < point_count; ++i) {
      if (tree->nearestKSearch(aligned_cloud.points[i], 1, nn_indices, nn_squared_distances) < 1) {
        continue;
      }
      const double squared_distance = static_cast<double>(nn_squared_distances[0]);
      if (squared_distance <= trim_distance_sq) {
        squared_distance_sum += squared_distance;
        ++scored;
      }
      if (squared_distance <= inlier_distance_sq) {
        ++inliers;
      }
    }
  }

  if (scored > 0) {
    quality.fitness_score = squared_distance_sum / static_cast<double>(scored);
  }
  quality.inlier_ratio =
    static_cast<double>(inliers) / static_cast<double>(aligned_cloud.size());
  return quality;
}

double PCLLocalization::integrateOdom(
  geometry_msgs::msg::Pose & pose, double t0, double t1) const
{
  // Replays the buffered body twists over (t0, t1] onto `pose`, using the same
  // convention the odom callback has always used: each sample's twist is held
  // over the interval that ends at that sample's stamp.
  double roll, pitch, yaw;
  tf2::Quaternion previous_quat_tf;
  tf2::fromMsg(pose.orientation, previous_quat_tf);
  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  double prev_t = t0;
  double applied_to = t0;
  bool applied = false;

  for (const auto & sample : odom_buffer_) {
    if (sample.t <= t0) {continue;}
    if (sample.t > t1) {break;}

    const double dt = sample.t - prev_t;
    prev_t = sample.t;
    if (dt <= 0.0 || dt > 1.0) {continue;}

    roll += sample.wx * dt;
    pitch += sample.wy * dt;
    yaw += sample.wz * dt;

    Eigen::Quaterniond quat_eig =
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Vector3d body_velocity{sample.vx, sample.vy, sample.vz};
    const Eigen::Vector3d delta_position = quat_eig.matrix() * dt * body_velocity;

    pose.position.x += delta_position.x();
    pose.position.y += delta_position.y();
    pose.position.z += delta_position.z();
    pose.orientation = tf2::toMsg(quat_eig);

    applied = true;
    applied_to = sample.t;
  }

  return applied ? applied_to : t0;
}

double PCLLocalization::deIntegrateOdom(
  geometry_msgs::msg::Pose & pose, double t0, double t1) const
{
  // Undoes exactly what integrateOdom(pose, t1, t0) would apply: the same
  // samples, paired with the same dt (each interval ends at its sample's
  // stamp, the first one measured from t1), replayed in reverse order and
  // with each step inverted -- position first, using the rotation that step
  // ended with, then the rotation itself.
  if (t1 >= t0) {return t0;}

  std::vector<std::pair<const OdomSample *, double>> steps;
  double prev_t = t1;
  for (const auto & sample : odom_buffer_) {
    if (sample.t <= t1) {continue;}
    if (sample.t > t0) {break;}
    const double dt = sample.t - prev_t;
    prev_t = sample.t;
    if (dt <= 0.0 || dt > 1.0) {continue;}
    steps.emplace_back(&sample, dt);
  }

  if (steps.empty()) {return t0;}

  double roll, pitch, yaw;
  tf2::Quaternion previous_quat_tf;
  tf2::fromMsg(pose.orientation, previous_quat_tf);
  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  for (auto it = steps.rbegin(); it != steps.rend(); ++it) {
    const OdomSample & sample = *it->first;
    const double dt = it->second;

    const Eigen::Quaterniond quat_eig =
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Vector3d body_velocity{sample.vx, sample.vy, sample.vz};
    const Eigen::Vector3d delta_position = quat_eig.matrix() * dt * body_velocity;

    pose.position.x -= delta_position.x();
    pose.position.y -= delta_position.y();
    pose.position.z -= delta_position.z();

    roll -= sample.wx * dt;
    pitch -= sample.wy * dt;
    yaw -= sample.wz * dt;
  }

  const Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  pose.orientation = tf2::toMsg(quat_eig);

  return t1;
}

bool PCLLocalization::extractPointTimeOffsets(
  const sensor_msgs::msg::PointCloud2 & msg, std::vector<float> & offsets) const
{
  const sensor_msgs::msg::PointField * time_field = nullptr;
  for (const auto & field : msg.fields) {
    if (field.name == "t" || field.name == "time" || field.name == "timestamp") {
      time_field = &field;
      break;
    }
  }
  if (!time_field) {return false;}

  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (point_count == 0) {return false;}
  offsets.assign(point_count, 0.0f);

  switch (time_field->datatype) {
    case sensor_msgs::msg::PointField::UINT32: {
        // The Ouster driver's `original` point type: nanoseconds since the
        // first column of this revolution, which is the instant header.stamp
        // names (lidar_packet_handler.h extrapolates the stamp back to it).
        sensor_msgs::PointCloud2ConstIterator<uint32_t> iter(msg, time_field->name);
        for (size_t i = 0; i < point_count; ++i, ++iter) {
          offsets[i] = static_cast<float>(*iter) * 1e-9f;
        }
        break;
      }
    case sensor_msgs::msg::PointField::FLOAT32: {
        // Velodyne-style: seconds, usually relative to the stamp already.
        const double stamp = rclcpp::Time(msg.header.stamp).seconds();
        sensor_msgs::PointCloud2ConstIterator<float> iter(msg, time_field->name);
        for (size_t i = 0; i < point_count; ++i, ++iter) {
          const double value = static_cast<double>(*iter);
          offsets[i] = static_cast<float>(value > 1e6 ? value - stamp : value);
        }
        break;
      }
    case sensor_msgs::msg::PointField::FLOAT64: {
        const double stamp = rclcpp::Time(msg.header.stamp).seconds();
        sensor_msgs::PointCloud2ConstIterator<double> iter(msg, time_field->name);
        for (size_t i = 0; i < point_count; ++i, ++iter) {
          const double value = *iter;
          offsets[i] = static_cast<float>(value > 1e6 ? value - stamp : value);
        }
        break;
      }
    default:
      return false;
  }

  return true;
}

bool PCLLocalization::deskewCloudWithOdom(
  pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  const std::vector<float> & offsets, double reference_time) const
{
  if (!cloud || cloud->points.size() != offsets.size() || odom_buffer_.empty()) {
    return false;
  }

  float sweep_span = 0.0f;
  for (const float offset : offsets) {
    if (std::isfinite(offset)) {sweep_span = std::max(sweep_span, offset);}
  }
  // A cloud whose points all share one stamp has nothing smeared into it.
  if (sweep_span < 1e-4f) {return false;}
  // Never extrapolate: the twists that cover this sweep have to be in hand,
  // or the "correction" is invention.
  if (odom_buffer_.front().t > reference_time ||
    odom_buffer_.back().t <
    reference_time + static_cast<double>(sweep_span) - kDeskewEndToleranceSec)
  {
    return false;
  }

  // The body pose at 32 instants across the sweep, each expressed relative to
  // where the body was at reference_time. Integrating each one from a fresh
  // identity keeps the dt pairing identical to integrateOdom's, which chaining
  // the calls would not: an interval with no odom sample inside it would
  // silently drop its share of the motion.
  constexpr int kDeskewBins = 32;
  std::array<Eigen::Affine3f, kDeskewBins + 1> bin_transform;
  for (int bin = 0; bin <= kDeskewBins; ++bin) {
    geometry_msgs::msg::Pose delta;
    delta.orientation.w = 1.0;
    const double bin_time = reference_time +
      static_cast<double>(sweep_span) * bin / kDeskewBins;
    integrateOdom(delta, reference_time, bin_time);
    Eigen::Affine3d delta_affine;
    tf2::fromMsg(delta, delta_affine);
    bin_transform[bin] = delta_affine.cast<float>();
  }

  const float bin_scale = kDeskewBins / sweep_span;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    auto & point = cloud->points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    int bin = static_cast<int>(std::lround(offsets[i] * bin_scale));
    bin = std::min(std::max(bin, 0), kDeskewBins);
    const Eigen::Vector3f corrected =
      bin_transform[bin] * Eigen::Vector3f(point.x, point.y, point.z);
    point.x = corrected.x();
    point.y = corrected.y();
    point.z = corrected.z();
  }

  return true;
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_) {return;}

  const double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  // Buffer first, unconditionally: cloudReceived needs these samples to replay
  // the scan-to-now interval onto the NDT result even for the very first
  // message, and even before an initial pose exists.
  odom_buffer_.push_back(
    OdomSample{
      current_odom_received_time,
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z});
  while (!odom_buffer_.empty() &&
    odom_buffer_.back().t - odom_buffer_.front().t > kOdomBufferSpanSec)
  {
    odom_buffer_.pop_front();
  }

  if (!initialpose_recieved_ || !corrent_pose_with_cov_stamped_ptr_) {
    RCLCPP_DEBUG(
      get_logger(),
      "Ignoring odometry until an initial pose is available");
    last_odom_received_time_ = current_odom_received_time;
    return;
  }

  const double previous_odom_received_time = last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;

  // First message after startup (or after the pose was re-anchored) has no
  // interval to integrate over yet.
  if (previous_odom_received_time <= 0.0) {return;}

  const double dt_odom = current_odom_received_time - previous_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  integrateOdom(
    corrent_pose_with_cov_stamped_ptr_->pose.pose,
    previous_odom_received_time, current_odom_received_time);
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
    tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

bool PCLLocalization::latLonToMapXY(double lat, double lon, double & map_x, double & map_y) const
{
  if (gnss_origin_lat_ == 0.0 && gnss_origin_lon_ == 0.0) {
    return false;
  }

  // Local flat-Earth (equirectangular) approximation around gnss_origin_*.
  // Good to a few cm over the hundreds-of-meters scale this map covers;
  // not meant for anything beyond a local recovery reference.
  constexpr double kEarthRadius = 6378137.0;
  const double origin_lat_rad = gnss_origin_lat_ * M_PI / 180.0;
  const double d_lat_rad = (lat - gnss_origin_lat_) * M_PI / 180.0;
  const double d_lon_rad = (lon - gnss_origin_lon_) * M_PI / 180.0;

  const double north = d_lat_rad * kEarthRadius;
  const double east = d_lon_rad * kEarthRadius * std::cos(origin_lat_rad);

  // Rotate ENU (east, north) into the map frame by the calibrated offset.
  const double cos_yaw = std::cos(gnss_to_map_yaw_offset_rad_);
  const double sin_yaw = std::sin(gnss_to_map_yaw_offset_rad_);
  map_x = east * cos_yaw - north * sin_yaw;
  map_y = east * sin_yaw + north * cos_yaw;
  return true;
}

void PCLLocalization::gnssReceived(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (!use_gnss_recovery_) {return;}

  if (msg->status.status < 0) {
    RCLCPP_DEBUG(get_logger(), "Ignoring gnss fix: no fix");
    return;
  }

  const double horizontal_covariance =
    std::max(msg->position_covariance[0], msg->position_covariance[4]);
  if (horizontal_covariance > gnss_max_covariance_) {
    RCLCPP_DEBUG(
      get_logger(), "Ignoring gnss fix: covariance %.3f exceeds max %.3f",
      horizontal_covariance, gnss_max_covariance_);
    return;
  }

  double map_x, map_y;
  if (!latLonToMapXY(msg->latitude, msg->longitude, map_x, map_y)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "gnss_origin_lat/gnss_origin_lon are not set; gnss recovery is disabled "
      "until they are calibrated for this map.");
    return;
  }

  last_gnss_map_x_ = map_x;
  last_gnss_map_y_ = map_y;
  last_gnss_stamp_ = rclcpp::Time(msg->header.stamp);
  gnss_fix_available_ = true;
}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg) {
    RCLCPP_WARN(get_logger(), "Received null point cloud message");
    return;
  }

  if (!map_recieved_ || !initialpose_recieved_) {return;}

  // Catch the odom buffer up before touching the scan. The executor hands over
  // one message per subscription per pass, so with a scan already waiting it
  // would take a single odom sample and come straight back here, leaving the
  // buffer short of the end of this sweep -- which is what made the deskew
  // below give up on most scans.
  if (use_odom_ && odom_sub_) {
    nav_msgs::msg::Odometry pending_odom;
    rclcpp::MessageInfo pending_info;
    while (odom_sub_->take(pending_odom, pending_info)) {
      odomReceived(std::make_shared<const nav_msgs::msg::Odometry>(pending_odom));
    }
  }
  // Per-scan record for debug_csv_path; written on every way out of this
  // callback, so rejected and unconverged scans are in the file too.
  const auto diag_t0 = std::chrono::steady_clock::now();
  auto diag_ms = [](std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
  auto diag_yaw = [](const geometry_msgs::msg::Quaternion & q) {
      return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    };
  struct Diag
  {
    double stamp{0}, age_ms{0}, odom_lead_ms{0}, prior_x{0}, prior_y{0}, prior_yaw{0};
    double pre_ms{0}, align_ms{0}, eval_ms{0}, fitness{-1}, inlier{-1};
    double ndt_x{0}, ndt_y{0}, ndt_z{0}, ndt_yaw{0}, correction{-1}, tp{-1};
    size_t n_src{0}; int deskewed{0}, walked_back{0}, iters{-1}, status{-1}, probe{0};
    double prior_inlier{-1};
  } diag;
  diag.stamp = rclcpp::Time(msg->header.stamp).seconds();
  diag.age_ms = (now().seconds() - diag.stamp) * 1e3;
  diag.odom_lead_ms = odom_buffer_.empty() ? -1e9 : (odom_buffer_.back().t - diag.stamp) * 1e3;
  struct DiagExit { std::function<void()> f; ~DiagExit() {if (f) {f();}} } diag_exit;
  if (debug_csv_.is_open()) {
    diag_exit.f = [&]() {
        const auto & pp = corrent_pose_with_cov_stamped_ptr_->pose.pose;
        debug_csv_ << std::fixed << std::setprecision(4) << diag.stamp << ',' << diag.age_ms << ','
                   << diag.odom_lead_ms << ',' << diag.n_src << ',' << diag.deskewed << ','
                   << diag.walked_back << ',' << diag.prior_x << ',' << diag.prior_y << ','
                   << diag.prior_yaw << ',' << diag.iters << ',' << diag.pre_ms << ','
                   << diag.align_ms << ',' << diag.eval_ms << ','
                   << diag_ms(diag_t0, std::chrono::steady_clock::now()) << ',' << diag.fitness << ','
                   << diag.inlier << ',' << diag.ndt_x << ',' << diag.ndt_y << ',' << diag.ndt_z << ','
                   << diag.ndt_yaw << ',' << diag.correction << ',' << diag.status << ','
                   << pp.position.x << ',' << pp.position.y << ',' << diag_yaw(pp.orientation) << ','
                   << rclcpp::Time(corrent_pose_with_cov_stamped_ptr_->header.stamp).seconds() << ','
                   << diag.tp << ',' << diag.prior_inlier << ',' << diag.probe << '\n';
        debug_csv_.flush();
      };
  }
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 2000, "cloudReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  // If your cloud is not robot-centric, convert to base_frame.
  if (msg->header.frame_id != base_frame_id_) {
    RCLCPP_DEBUG(
        this->get_logger(), "Transforming point cloud from %s to %s",
        msg->header.frame_id.c_str(), base_frame_id_.c_str());
    geometry_msgs::msg::TransformStamped base_to_lidar_stamped;
    try {
      base_to_lidar_stamped = tfbuffer_.lookupTransform(
          base_frame_id_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
      return;
    }

    Eigen::Matrix4f initial_transformation =
      tf2::transformToEigen(base_to_lidar_stamped.transform).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud, initial_transformation);
    cloud_ptr = transformed_cloud;
  }

  const double scan_time = rclcpp::Time(msg->header.stamp).seconds();

  // Straighten the revolution before anything else looks at it. Every point
  // was taken at a different instant while the cart kept moving, so the cloud
  // NDT sees is stretched by up to v * scan_period along the path -- 0.2 m at
  // 2 m/s, 0.4 m at 4 m/s. Undoing it with the odom twists is what keeps the
  // registration noise from growing with speed.
  bool deskewed = false;
  if (use_odom_ && use_scan_deskew_) {
    std::vector<float> point_time_offsets;
    if (extractPointTimeOffsets(*msg, point_time_offsets) &&
      point_time_offsets.size() == cloud_ptr->points.size())
    {
      deskewed = deskewCloudWithOdom(cloud_ptr, point_time_offsets, scan_time);
      diag.deskewed = deskewed ? 1 : 0;
      if (!deskewed) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Scan deskew skipped: the odom buffer does not cover this sweep.");
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Scan deskew skipped: no usable per-point time field on %s (the Ouster "
        "driver needs point_type: original).", msg->header.frame_id.c_str());
    }
  }

  if (!deskewed && use_imu_) {
    lidar_undistortion_.adjustDistortion(cloud_ptr, scan_time);
  }

  // Range-gate BEFORE voxelising. The Ouster returns points out to ~120 m and
  // everything outside [scan_min_range_, scan_max_range_] is discarded anyway,
  // so filtering second meant building a voxel grid over the full sensor range
  // and then throwing most of the result away. Squared radii avoid a sqrt per
  // point, and NaN returns fail the comparison and drop out here.
  pcl::PointCloud<pcl::PointXYZI>::Ptr range_filtered_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>());
  range_filtered_cloud_ptr->points.reserve(cloud_ptr->points.size());
  const double min_range_sq = scan_min_range_ * scan_min_range_;
  const double max_range_sq = scan_max_range_ * scan_max_range_;
  // Optional height notch, in base_frame z. Vehicles parked in a lot the map
  // was surveyed empty occupy a known band -- roughly the ground up to roof
  // height -- so dropping that band feeds NDT only the structure that was
  // there on both days. Off by default (min == max).
  const bool exclude_height_band = scan_exclude_z_max_ > scan_exclude_z_min_;
  for (const auto & p : cloud_ptr->points) {
    if (exclude_height_band &&
      p.z > scan_exclude_z_min_ && p.z < scan_exclude_z_max_)
    {
      continue;
    }
    const double range_sq = static_cast<double>(p.x) * static_cast<double>(p.x) +
      static_cast<double>(p.y) * static_cast<double>(p.y);
    if (min_range_sq < range_sq && range_sq < max_range_sq) {
      range_filtered_cloud_ptr->points.push_back(p);
    }
  }
  range_filtered_cloud_ptr->width = range_filtered_cloud_ptr->points.size();
  range_filtered_cloud_ptr->height = 1;
  range_filtered_cloud_ptr->is_dense = false;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(range_filtered_cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);
  registration_->setInputSource(filtered_cloud_ptr);
  diag.n_src = filtered_cloud_ptr->size();

  // corrent_pose_ has already been dead-reckoned to the newest odom sample,
  // but this scan is a snapshot of scan_time. Handing NDT the newer pose
  // starts the search roughly v * latency away from the answer -- ~0.3 m at
  // 2 m/s, and it keeps growing until it approaches ndt_resolution and the
  // search can settle into the neighbouring voxel. It also inflates the
  // correction gate below by the same amount, which is how a gate tuned at
  // walking pace starts rejecting good matches once the cart drives faster.
  // Walking the prior back to scan time makes both speed-independent.
  geometry_msgs::msg::Pose prior_pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  if (use_odom_ && use_scan_time_init_guess_ &&
    scan_time < last_odom_received_time_ &&
    last_odom_received_time_ - scan_time <= kOdomBufferSpanSec)
  {
    deIntegrateOdom(prior_pose, last_odom_received_time_, scan_time);
    diag.walked_back = 1;
  }
  diag.prior_x = prior_pose.position.x;
  diag.prior_y = prior_pose.position.y;
  diag.prior_yaw = diag_yaw(prior_pose.orientation);

  Eigen::Affine3d affine;
  tf2::fromMsg(prior_pose, affine);

  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  const auto diag_t1 = std::chrono::steady_clock::now();
  diag.pre_ms = diag_ms(diag_t0, diag_t1);
  registration_->align(*output_cloud, init_guess);
  const auto diag_t2 = std::chrono::steady_clock::now();
  diag.align_ms = diag_ms(diag_t1, diag_t2);
  rclcpp::Time time_align_end = system_clock.now();
  if (auto ndt_omp = boost::dynamic_pointer_cast<
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_))
  {
    diag.iters = ndt_omp->getFinalNumIteration();
    diag.tp = ndt_omp->getTransformationProbability();
  }

  bool has_converged = registration_->hasConverged();
  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    diag.status = 3;
    return;
  }

  MatchQuality match = evaluateMatch(*output_cloud);
  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

  // A match that explains too little of the scan is usually NDT sitting in the
  // neighbouring optimum along the road: on a straight lined with similar
  // structure the score has a second basin a metre or two up or down the
  // path, and once the pose is in it every scan converges there in a few
  // iterations with a small correction, so nothing ever looks wrong. Retry
  // from one and two probe_offsets ahead and behind and keep whichever
  // explains the scan clearly better.
  if (probe_inlier_threshold_ > 0.0 && match.inlier_ratio < probe_inlier_threshold_ &&
    scan_time - last_probe_time_ >= probe_min_interval_sec_)
  {
    last_probe_time_ = scan_time;
    diag.probe = 1;
    const Eigen::Vector3f heading = final_transformation.block<3, 1>(0, 0);
    const Eigen::Matrix4f tracked_transformation = final_transformation;
    const float probe_step = static_cast<float>(probe_offset_m_);
    for (const float offset : {probe_step, -probe_step, 2.0f * probe_step, -2.0f * probe_step}) {
      Eigen::Matrix4f probe_guess = tracked_transformation;
      probe_guess.block<3, 1>(0, 3) += offset * heading;
      pcl::PointCloud<pcl::PointXYZI>::Ptr probe_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      registration_->align(*probe_cloud, probe_guess);
      if (!registration_->hasConverged()) {continue;}
      const MatchQuality probe_match = evaluateMatch(*probe_cloud);
      if (probe_match.inlier_ratio >= match.inlier_ratio + probe_accept_inlier_gain_) {
        RCLCPP_WARN(
          get_logger(),
          "Along-track probe found a better match %.2f m away: %.0f%% -> %.0f%% of the scan "
          "explained.",
          (registration_->getFinalTransformation().block<3, 1>(0, 3) -
          tracked_transformation.block<3, 1>(0, 3)).norm(),
          100.0 * match.inlier_ratio, 100.0 * probe_match.inlier_ratio);
        match = probe_match;
        final_transformation = registration_->getFinalTransformation();
        diag.probe = 2;
      }
    }
  }
  const double fitness_score = match.fitness_score;
  diag.eval_ms = diag_ms(diag_t2, std::chrono::steady_clock::now());
  diag.fitness = fitness_score;
  diag.inlier = match.inlier_ratio;

  if (match_inlier_ratio_pub_ && match_inlier_ratio_pub_->is_activated()) {
    std_msgs::msg::Float64 inlier_ratio_msg;
    inlier_ratio_msg.data = match.inlier_ratio;
    match_inlier_ratio_pub_->publish(inlier_ratio_msg);
  }

  const bool unexplained_scan =
    min_inlier_ratio_ > 0.0 && match.inlier_ratio < min_inlier_ratio_;
  if (fitness_score > score_threshold_ || unexplained_scan) {
    RCLCPP_WARN(
      get_logger(),
      "Poor match: fitness %.3f (limit %.3f), %.0f%% of the scan found a map "
      "correspondence%s.",
      fitness_score, score_threshold_, 100.0 * match.inlier_ratio,
      unexplained_scan ? " -- too little of the scan is in the map" : "");
    ++consecutive_bad_fitness_count_;
  } else {
    consecutive_bad_fitness_count_ = 0;
  }

  // Outlier gate. init_guess is where the odom prior says the vehicle is, so
  // the norm below is how far NDT wants to move it in a single scan period --
  // a distance the cart physically cannot have covered unnoticed. Accepting
  // one of those is exactly the "jumped 0.6-1.7 m on recovery" failure; the
  // odom prediction is a better pose than a confident wrong match. The streak
  // escape hatch keeps a genuinely displaced vehicle (pushed, lifted, or
  // recovering from a long dropout) from being locked out of relocalising.
  const double correction_distance =
    (final_transformation.block<3, 1>(0, 3) - init_guess.block<3, 1>(0, 3)).norm();
  diag.correction = correction_distance;
  diag.ndt_x = final_transformation(0, 3);
  diag.ndt_y = final_transformation(1, 3);
  diag.ndt_z = final_transformation(2, 3);
  diag.ndt_yaw = std::atan2(final_transformation(1, 0), final_transformation(0, 0));
  diag.status = 0;
  // ... unless the far pose explains the scan clearly better than the prior
  // does. Distance alone cannot tell a wrong match from a wrong prior: when
  // the tracked pose has slipped, the correct answer IS far away, and holding
  // the prior for ten scans keeps the cart lost for a second or more per
  // attempt (2026-10-02 bag: a 0.87-inlier match rejected in favour of a
  // 0.67-inlier prior for 12 s). Scoring the scan at the prior makes the gate
  // compare evidence instead.
  bool correction_is_improvement = false;
  if (max_correction_distance_ > 0.0 && correction_distance > max_correction_distance_ &&
    correction_accept_inlier_gain_ > 0.0)
  {
    pcl::PointCloud<pcl::PointXYZI> prior_cloud;
    pcl::transformPointCloud(*filtered_cloud_ptr, prior_cloud, init_guess);
    const MatchQuality prior_match = evaluateMatch(prior_cloud);
    diag.prior_inlier = prior_match.inlier_ratio;
    correction_is_improvement =
      match.inlier_ratio >= prior_match.inlier_ratio + correction_accept_inlier_gain_;
    if (correction_is_improvement) {
      RCLCPP_WARN(
        get_logger(),
        "Accepting a %.2f m NDT correction: it explains %.0f%% of the scan against %.0f%% "
        "at the odom prediction.",
        correction_distance, 100.0 * match.inlier_ratio, 100.0 * prior_match.inlier_ratio);
      diag.status = 4;
    }
  }
  if (max_correction_distance_ > 0.0 && correction_distance > max_correction_distance_ &&
    !correction_is_improvement)
  {
    ++consecutive_rejected_count_;
    ++consecutive_bad_fitness_count_;
    if (consecutive_rejected_count_ < max_correction_reject_streak_) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected NDT result: it would move the pose %.2f m in one scan "
        "(limit %.2f m, fitness %.3f, rejection %d/%d). Holding the odom prediction.",
        correction_distance, max_correction_distance_, fitness_score,
        consecutive_rejected_count_, max_correction_reject_streak_);
      last_scan_ptr_ = msg;
      diag.status = 1;
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Accepting a %.2f m NDT correction after %d consecutive rejections; "
      "treating the vehicle as genuinely displaced.",
      correction_distance, consecutive_rejected_count_);
    diag.status = 2;
  }
  consecutive_rejected_count_ = 0;

  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  geometry_msgs::msg::Pose ndt_pose;
  ndt_pose.position.x = static_cast<double>(final_transformation(0, 3));
  ndt_pose.position.y = static_cast<double>(final_transformation(1, 3));
  ndt_pose.position.z = static_cast<double>(final_transformation(2, 3));
  ndt_pose.orientation = quat_msg;

  // The pose above is where the vehicle was at msg->header.stamp, but a scan
  // period plus the registration time has passed since. Writing it back raw
  // discards every odom sample that arrived in between and drags the published
  // pose backwards by roughly v * latency on every single update -- the
  // sawtooth that shows up as sub-second 0.5-0.9 m steps in /pcl_pose. Replay
  // that interval so the published pose is current instead of retrospective.
  double pose_time = scan_time;
  if (use_odom_ && use_scan_latency_compensation_ && !odom_buffer_.empty()) {
    const double compensation_end = std::min(
      odom_buffer_.back().t, pose_time + max_latency_compensation_sec_);
    pose_time = integrateOdom(ndt_pose, pose_time, compensation_end);
  }

  // pose_time is normally the newest buffered odom stamp, i.e. the same instant
  // last_odom_received_time_ holds, so the next odomReceived picks up exactly
  // where this left off. Only a latency past max_latency_compensation_sec_
  // leaves a gap, and a prior that stale is not worth extrapolating anyway.
  corrent_pose_with_cov_stamped_ptr_->header.stamp = rclcpp::Time(
    static_cast<int64_t>(pose_time * 1e9), RCL_ROS_TIME);
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose = ndt_pose;

  // Observational cross-check only: publish how far NDT's pose sits from an
  // independent RTK fix, without acting on it. Fitness score alone was too
  // noisy to gate pose acceptance on (rejected too often during normal
  // driving); watching the NDT/GNSS gap first lets a real threshold be
  // chosen from data instead of guessing.
  if (use_gnss_recovery_ && gnss_fix_available_) {
    const double gnss_age_sec = (rclcpp::Time(msg->header.stamp) - last_gnss_stamp_).seconds();
    if (gnss_age_sec >= 0.0 && gnss_age_sec <= gnss_max_age_sec_) {
      const double dx = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x - last_gnss_map_x_;
      const double dy = corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y - last_gnss_map_y_;
      const double disagreement_m = std::sqrt(dx * dx + dy * dy);

      if (gnss_ndt_disagreement_pub_ && gnss_ndt_disagreement_pub_->is_activated()) {
        std_msgs::msg::Float64 disagreement_msg;
        disagreement_msg.data = disagreement_m;
        gnss_ndt_disagreement_pub_->publish(disagreement_msg);
      }

      if (disagreement_m >= gnss_ndt_disagreement_warn_m_) {
        RCLCPP_WARN(
          get_logger(),
          "NDT/GNSS disagreement %.2f m (>= %.2f m warn threshold): "
          "ndt=(%.3f, %.3f) gnss=(%.3f, %.3f), fitness=%.3f",
          disagreement_m, gnss_ndt_disagreement_warn_m_,
          corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x,
          corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y,
          last_gnss_map_x_, last_gnss_map_y_, fitness_score);
      }
    }
  }

  if (use_gnss_recovery_ && gnss_fix_available_ &&
    consecutive_bad_fitness_count_ >= gnss_lost_scan_threshold_)
  {
    const double gnss_age_sec = (rclcpp::Time(msg->header.stamp) - last_gnss_stamp_).seconds();
    if (gnss_age_sec >= 0.0 && gnss_age_sec <= gnss_max_age_sec_) {
      RCLCPP_WARN(
        get_logger(),
        "NDT lost for %d scans; re-anchoring pose from gnss fix at x=%.3f, y=%.3f",
        consecutive_bad_fitness_count_, last_gnss_map_x_, last_gnss_map_y_);
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = last_gnss_map_x_;
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = last_gnss_map_y_;
      consecutive_bad_fitness_count_ = 0;
    }
  }

  // publish here if timer is not enabled

  if (!enable_timer_publishing_){
    // Everything below is stamped with the pose's own validity time, which the
    // latency compensation moved forward from the scan stamp.
    const auto pose_stamp = corrent_pose_with_cov_stamped_ptr_->header.stamp;

    pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

    geometry_msgs::msg::TransformStamped map_to_base_link_stamped;
    map_to_base_link_stamped.header.stamp = pose_stamp;
    map_to_base_link_stamped.header.frame_id = global_frame_id_;
    map_to_base_link_stamped.child_frame_id = base_frame_id_;
    map_to_base_link_stamped.transform.translation.x =
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x;
    map_to_base_link_stamped.transform.translation.y =
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y;
    map_to_base_link_stamped.transform.translation.z =
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z;
    map_to_base_link_stamped.transform.rotation =
      corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation;
    if (!enable_map_odom_tf_) {
      broadcaster_.sendTransform(map_to_base_link_stamped);
    } else {
      tf2::Transform map_to_base_link_tf;
      tf2::fromMsg(map_to_base_link_stamped.transform, map_to_base_link_tf);

      geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
      try {
        odom_to_base_link_msg = tfbuffer_.lookupTransform(
          odom_frame_id_, base_frame_id_, pose_stamp, rclcpp::Duration::from_seconds(0.1));
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(), "Could not get transform %s to %s: %s",
          base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
        return;
      }
      tf2::Transform odom_to_base_link_tf;
      tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

      tf2::Transform map_to_odom_tf = map_to_base_link_tf * odom_to_base_link_tf.inverse();
      geometry_msgs::msg::TransformStamped map_to_odom_stamped;
      map_to_odom_stamped.header.stamp = pose_stamp;
      map_to_odom_stamped.header.frame_id = global_frame_id_;
      map_to_odom_stamped.child_frame_id = odom_frame_id_;
      map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
      broadcaster_.sendTransform(map_to_odom_stamped);
    }

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
    pose_stamped_ptr->header.stamp = pose_stamp;
    pose_stamped_ptr->header.frame_id = global_frame_id_;
    pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped_ptr);
    if (path_max_length_ > 0 && path_ptr_->poses.size() > static_cast<size_t>(path_max_length_)) {
      path_ptr_->poses.erase(path_ptr_->poses.begin());
    }
    path_pub_->publish(*path_ptr_);
  }

  last_scan_ptr_ = msg;

  if (enable_debug_) {
    std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
    std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() <<
      "[sec]" << std::endl;
    std::cout << "has converged: " << has_converged << std::endl;
    std::cout << "fitness score: " << fitness_score << std::endl;
    std::cout << "inlier ratio: " << match.inlier_ratio << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;
    /* delta_angle check
     * trace(RotationMatrix) = 2(cos(theta) + 1)
     */
    double init_cos_angle = 0.5 *
      (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
      (final_transformation.coeff(0,
      0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
  }
}

void PCLLocalization::timerPublishPose()
{
  if (!corrent_pose_with_cov_stamped_ptr_) {return;}
  geometry_msgs::msg::PoseWithCovarianceStamped pose_copy = *corrent_pose_with_cov_stamped_ptr_;
  pose_copy.header.stamp = now();

  geometry_msgs::msg::PoseStamped stamped;
  stamped.header = pose_copy.header;
  stamped.header.frame_id = global_frame_id_;
  stamped.pose = pose_copy.pose.pose;
  path_ptr_->poses.push_back(stamped);
  if (path_max_length_ > 0 && path_ptr_->poses.size() > static_cast<size_t>(path_max_length_)) {
    path_ptr_->poses.erase(path_ptr_->poses.begin());
  }

  nav_msgs::msg::Path path_copy = *path_ptr_;

  pose_pub_->publish(pose_copy);
  path_pub_->publish(path_copy);

  geometry_msgs::msg::TransformStamped map_to_base_link_stamped;
  map_to_base_link_stamped.header.stamp = pose_copy.header.stamp;
  map_to_base_link_stamped.header.frame_id = global_frame_id_;
  map_to_base_link_stamped.child_frame_id = base_frame_id_;
  map_to_base_link_stamped.transform.translation.x = pose_copy.pose.pose.position.x;
  map_to_base_link_stamped.transform.translation.y = pose_copy.pose.pose.position.y;
  map_to_base_link_stamped.transform.translation.z = pose_copy.pose.pose.position.z;
  map_to_base_link_stamped.transform.rotation = pose_copy.pose.pose.orientation;

  if (!enable_map_odom_tf_) {
    broadcaster_.sendTransform(map_to_base_link_stamped);
  } else {
    tf2::Transform map_to_base_link_tf;
    tf2::fromMsg(map_to_base_link_stamped.transform, map_to_base_link_tf);

    geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
    try {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, pose_copy.header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not get transform %s to %s: %s",
        base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
      return;
    }
    tf2::Transform odom_to_base_link_tf;
    tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

    tf2::Transform map_to_odom_tf = map_to_base_link_tf * odom_to_base_link_tf.inverse();
    geometry_msgs::msg::TransformStamped map_to_odom_stamped;
    map_to_odom_stamped.header.stamp = pose_copy.header.stamp;
    map_to_odom_stamped.header.frame_id = global_frame_id_;
    map_to_odom_stamped.child_frame_id = odom_frame_id_;
    map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
    
    broadcaster_.sendTransform(map_to_odom_stamped);
  }
}
