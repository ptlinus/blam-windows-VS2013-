#ifndef LASER_LOOP_CLOSURE_H
#define LASER_LOOP_CLOSURE_H

#include <geometry_utils/Matrix3x3.h>
#include <geometry_utils/Transform3.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <point_cloud_filter/PointCloudFilter.h>

#include <map>
#include <vector>
#include"common.h"

class LaserLoopClosure {
 public:
  LaserLoopClosure();
  ~LaserLoopClosure();

  bool Initialize();
  typedef geometry_utils::MatrixNxNBase<double, 6> Mat66;
  typedef pcl::PointCloud<pcl::POINT_TYPE> PointCloud;
  bool AddBetweenFactor(const geometry_utils::Transform3& delta,
                        const Mat66& covariance, unsigned int* key);
  bool AddKeyScanPair(unsigned int key, const PointCloud::ConstPtr& scan);
  bool FindLoopClosures(unsigned int key,
                        std::vector<unsigned int>* closure_keys);
  void GetMaximumLikelihoodPoints(PointCloud* map);
  geometry_utils::Transform3 GetLastPose() const;
  void PublishPoseGraph();

 private:
  bool LoadParameters();
  geometry_utils::Transform3 ToGu(const gtsam::Pose3& pose) const;
  gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;

  typedef gtsam::noiseModel::Gaussian Gaussian;
  typedef gtsam::noiseModel::Diagonal Diagonal;
  Mat66 ToGu(const Gaussian::shared_ptr& covariance) const;
  Gaussian::shared_ptr ToGtsam(const Mat66& covariance) const;

  gtsam::PriorFactor<gtsam::Pose3> MakePriorFactor(
      const gtsam::Pose3& pose, const Diagonal::shared_ptr& covariance);
  gtsam::BetweenFactor<gtsam::Pose3> MakeBetweenFactor(
      const gtsam::Pose3& pose, const Gaussian::shared_ptr& covariance);

  bool PerformICP(const PointCloud::ConstPtr& scan1,
                  const PointCloud::ConstPtr& scan2,
                  const geometry_utils::Transform3& pose1,
                  const geometry_utils::Transform3& pose2,
                  geometry_utils::Transform3* delta, Mat66* covariance);

  // Node name.
  std::string name_;

  // Keep a list of keyed laser scans and keyed timestamps.
  std::map<unsigned int, PointCloud::ConstPtr> keyed_scans_;
//  std::map<unsigned int, ros::Time> keyed_stamps_;

  // Aggregate odometry until we can update the pose graph.
  gtsam::Pose3 odometry_;

  // Pose graph and ISAM2 parameters.
  bool check_for_loop_closures_;
  unsigned int key_;
  unsigned int last_closure_key_;
  unsigned int relinearize_interval_;
  unsigned int skip_recent_poses_;
  unsigned int poses_before_reclosing_;
  double translation_threshold_;
  double proximity_threshold_;
  double max_tolerable_fitness_;

  // ICP parameters.
  double icp_ransac_thresh_;
  double icp_tf_epsilon_;
  double icp_corr_dist_;
  unsigned int icp_iterations_;

  // ISAM2 optimizer object, and best guess pose values.
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values values_;

  // Frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  typedef std::pair<unsigned int, unsigned int> Edge;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;

  // For filtering laser scans prior to ICP.
  PointCloudFilter filter_;
};

#endif
