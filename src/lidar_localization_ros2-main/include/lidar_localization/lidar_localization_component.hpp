#include <array>
#include <chrono>
#include <deque>
#include <fstream>
#include <vector>
#include <limits>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/ply_io.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include "lidar_localization/lidar_undistortion.hpp"

using namespace std::chrono_literals;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PCLLocalization(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  void initializeParameters();
  void initializePubSub();
  void initializeRegistration();
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void gnssReceived(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  /// How well the aligned scan is explained by the map.
  struct MatchQuality
  {
    /// Mean squared distance to the nearest map point, over correspondences
    /// closer than fitness_max_correspondence_distance_ (all of them when that
    /// is disabled). Same units and meaning as score_threshold.
    double fitness_score{std::numeric_limits<double>::max()};
    /// Fraction of scan points that found a map point within that distance.
    /// Drops when the scan contains things the map does not -- parked cars in
    /// a lot that was empty at survey time, scaffolding, a market stall -- and
    /// also when the vehicle is genuinely lost. Unlike the raw fitness score
    /// it says WHY the match is poor.
    double inlier_ratio{0.0};
  };
  MatchQuality evaluateMatch(const pcl::PointCloud<pcl::PointXYZI> & aligned_cloud) const;

  /// Dead-reckon `pose` forward over the buffered odom twists in (t0, t1].
  /// Returns the timestamp actually integrated up to (t0 if nothing applied).
  double integrateOdom(geometry_msgs::msg::Pose & pose, double t0, double t1) const;
  /// Exact inverse: walk `pose` back from t0 to t1 (t1 < t0) over the same
  /// buffered twists, so a pose that has been dead-reckoned to now can be
  /// asked "where was the vehicle when this scan started". Returns the
  /// timestamp reached (t0 if nothing applied).
  double deIntegrateOdom(geometry_msgs::msg::Pose & pose, double t0, double t1) const;
  /// Per-point acquisition times, in seconds after the cloud's header stamp
  /// and in point order. False when the cloud carries no usable time field.
  bool extractPointTimeOffsets(
    const sensor_msgs::msg::PointCloud2 & msg, std::vector<float> & offsets) const;
  /// Undo the motion smeared into one revolution: every point is moved onto
  /// the body pose held at `reference_time` using the buffered odom twists.
  /// False when the odom buffer does not cover the sweep.
  bool deskewCloudWithOdom(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const std::vector<float> & offsets, double reference_time) const;
  bool latLonToMapXY(double lat, double lon, double & map_x, double & map_y) const;

  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    initial_map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr
    imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr
    gnss_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr
    gnss_ndt_disagreement_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr
    match_inlier_ratio_pub_;

  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr corrent_pose_with_cov_stamped_ptr_;
  nav_msgs::msg::Path::SharedPtr path_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_scan_ptr_;

  bool map_recieved_{false};
  bool initialpose_recieved_{false};

  // parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string registration_method_;
  double scan_max_range_;
  double scan_min_range_;
  // Height band (in base_frame z) to drop from every scan, for structure that
  // exists on the road but not in the map. Equal values disable it.
  double scan_exclude_z_min_;
  double scan_exclude_z_max_;
  // Correspondences beyond this distance [m] are treated as "not in the map"
  // and left out of the fitness score instead of inflating it. 0 disables the
  // trimming and restores the plain PCL fitness score.
  double fitness_max_correspondence_distance_;
  // Below this inlier ratio the scan counts as unexplained by the map. 0 means
  // observe and publish only, never act.
  double min_inlier_ratio_;
  double scan_period_;
  double score_threshold_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  bool use_pcd_map_{false};
  std::string map_path_;
  bool set_initial_pose_{false};
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;

  bool use_odom_{false};
  double last_odom_received_time_{0.0};
  bool use_imu_{false};
  bool enable_debug_{false};
  bool enable_map_odom_tf_{false};

  int ndt_num_threads_;
  int ndt_max_iterations_;
  int path_max_length_;

  double pose_publish_frequency_;
  bool enable_timer_publishing_{false};

  // Scan-latency compensation. align() returns the pose the vehicle held at
  // msg->header.stamp (the start of the revolution), but by the time it
  // returns, one scan period plus the registration time has passed and the
  // odom prior has already dead-reckoned past it. Writing the raw NDT result
  // back therefore throws that motion away and snaps the published pose
  // backwards by roughly v * latency every update. The buffer keeps the odom
  // twists needed to replay that interval onto the NDT result.
  struct OdomSample
  {
    double t;
    double vx, vy, vz;
    double wx, wy, wz;
  };
  std::deque<OdomSample> odom_buffer_;
  bool use_scan_latency_compensation_{true};
  // Motion compensation within the revolution itself. The Ouster sweeps for a
  // full scan_period, so at v the last point of a scan is taken v*scan_period
  // further down the road than the first one, and NDT is handed a cloud that
  // is stretched along the direction of travel. That error is proportional to
  // speed -- which is exactly the shape of the noise measured in the
  // 2026-09-09 bags -- and the odom buffer already holds the twists needed to
  // straighten it out. Uses the per-point `t` field (point_type: original on
  // the driver side), never the IMU: see use_imu_ for why that one is off.
  bool use_scan_deskew_{true};
  // Hand NDT a prior taken at the scan's own timestamp rather than the
  // dead-reckoned present. Keeps both the search start and the correction
  // gate below independent of vehicle speed.
  bool use_scan_time_init_guess_{true};
  double max_latency_compensation_sec_;

  // Outlier gate: NDT converging onto the wrong part of the map shows up as a
  // correction the vehicle could not physically have needed since the last
  // scan. Rejecting those keeps the odom prediction instead of a teleport.
  double max_correction_distance_;
  int max_correction_reject_streak_;
  int consecutive_rejected_count_{0};

  // imu
  LidarUndistortion lidar_undistortion_;

  // gnss recovery: falls back to an RTK fix to re-anchor the pose once NDT
  // has been failing to match for too many consecutive scans, since NDT
  // itself has no way to tell "converged on the wrong place" from "lost".
  bool use_gnss_recovery_{false};
  std::string gnss_topic_;
  double gnss_origin_lat_;
  double gnss_origin_lon_;
  double gnss_to_map_yaw_offset_rad_;
  double gnss_max_covariance_;
  double gnss_max_age_sec_;
  int gnss_lost_scan_threshold_;
  double gnss_ndt_disagreement_warn_m_;
  int consecutive_bad_fitness_count_{0};
  bool gnss_fix_available_{false};
  double last_gnss_map_x_{0.0};
  double last_gnss_map_y_{0.0};
  rclcpp::Time last_gnss_stamp_;

  // A correction past max_correction_distance_ is still accepted when the NDT
  // pose explains at least this much more of the scan (inlier ratio) than the
  // odom prediction does. 0 keeps the distance-only gate.
  double correction_accept_inlier_gain_{0.0};
  // Along-track probe: below this inlier ratio, retry NDT from +-1 and +-2
  // probe_offset_m_ along the heading (at most once per
  // probe_min_interval_sec_) and switch to a result that gains at least
  // probe_accept_inlier_gain_. 0 disables it.
  double probe_inlier_threshold_{0.0};
  double probe_offset_m_{1.5};
  double probe_min_interval_sec_{1.0};
  double probe_accept_inlier_gain_{0.08};
  double last_probe_time_{0.0};
  // One CSV row per scan (timing, prior, NDT result, gate decision) when set.
  std::string debug_csv_path_;
  std::ofstream debug_csv_;

  rclcpp::TimerBase::SharedPtr pose_publish_timer_;
  void timerPublishPose();
};
