#include <point_cloud_odometry/PointCloudOdometry.h>
//#include <parameter_utils/ParameterUtils.h>
#include <parameter_utils/slamBase.h>


#include <pcl/registration/gicp.h>
#include"common.h"

namespace gu = geometry_utils;
//namespace gr = gu::ros;
//namespace pu = parameter_utils;

using pcl::copyPointCloud;
using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::POINT_TYPE;

PointCloudOdometry::PointCloudOdometry() : initialized_(false) {
  query_.reset(new PointCloud);
  reference_.reset(new PointCloud);
}

PointCloudOdometry::~PointCloudOdometry() {}

bool PointCloudOdometry::Initialize(/*const ros::NodeHandle& n*/) {
	if (!LoadParameters()) {

		return false;
	}
	return true;
}

bool PointCloudOdometry::LoadParameters() {
	ParameterReader pd;
	fixed_frame_id_ = pd.getData("fixed");
	odometry_frame_id_ = pd.getData("odometry");

	// Load initial position.
	double init_x = 0.0, init_y = 0.0, init_z = 0.0;
	double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
	init_x = atof(pd.getData("position_x").c_str());
	init_y = atof(pd.getData("position_y").c_str());
	init_z = atof(pd.getData("position_z").c_str());

	init_roll = atof(pd.getData("orientation_roll").c_str());
	init_pitch = atof(pd.getData("orientation_pitch").c_str());
	init_yaw = atof(pd.getData("orientation_yaw").c_str());
	gu::Transform3 init;
	init.translation = gu::Vec3(init_x, init_y, init_z);
	init.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);
	integrated_estimate_ = init;

	params_.icp_tf_epsilon = atof(pd.getData("tf_epsilon").c_str());
	params_.icp_corr_dist = atof(pd.getData("corr_dist").c_str());
	params_.icp_iterations = atoi(pd.getData("iterations").c_str());

	transform_thresholding_ = atoi(pd.getData("transform_thresholding").c_str());
	max_translation_ = atof(pd.getData("max_translation").c_str());
	max_rotation_ = atof(pd.getData("max_rotation").c_str());

	return true;
}


bool PointCloudOdometry::UpdateEstimate(const PointCloud& points) {
  if (!initialized_) {
    copyPointCloud(points, *query_);
    initialized_ = true;
    return false;
  }
  copyPointCloud(*query_, *reference_);
  copyPointCloud(points, *query_);
  return UpdateICP();
}

const gu::Transform3& PointCloudOdometry::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const gu::Transform3& PointCloudOdometry::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

bool PointCloudOdometry::GetLastPointCloud(PointCloud::Ptr& out) const {
  if (!initialized_ || query_ == NULL) {
    return false;
  }

  out = query_;
  return true;
}

bool PointCloudOdometry::UpdateICP() {
  // Compute the incremental transformation.
	GeneralizedIterativeClosestPoint<POINT_TYPE, POINT_TYPE> icp;
  icp.setTransformationEpsilon(params_.icp_tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.icp_corr_dist);
  icp.setMaximumIterations(params_.icp_iterations);
  icp.setRANSACIterations(0);

  icp.setInputSource(query_);
  icp.setInputTarget(reference_);

  PointCloud unused_result;
  icp.align(unused_result);

  const Eigen::Matrix4f T = icp.getFinalTransformation();

  // Update pose estimates.
  incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  incremental_estimate_.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                            T(1, 0), T(1, 1), T(1, 2),
                                            T(2, 0), T(2, 1), T(2, 2));

  // Only update if the incremental transform is small enough.
  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ =
        gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } else {
    
  }


  return true;
}
