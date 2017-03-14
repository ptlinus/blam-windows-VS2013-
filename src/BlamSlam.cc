#include <blam_slam/BlamSlam.h>
#include <geometry_utils/Transform3.h>
//#include <parameter_utils/ParameterUtils.h>

//namespace pu = parameter_utils;
namespace gu = geometry_utils;


BlamSlam::BlamSlam() {
}

BlamSlam::~BlamSlam() {}



bool BlamSlam::Initialize() {

	if (!filter_.Initialize()) {
		return false;
	}

	if (!odometry_.Initialize()) {
		return false;
	}

	if (!loop_closure_.Initialize()) {
	return false;
	}

	if (!localization_.Initialize()) {
		return false;
	}

	if (!mapper_.Initialize()) {
		return false;
	}
	return true;
}


bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr& scan,
	bool* new_keyframe) {
	if (new_keyframe == NULL) {
		return false;
	}

	// Add the new pose to the pose graph.
	unsigned int pose_key;
	gu::MatrixNxNBase<double, 6> covariance;
	covariance.Zeros();
	for (int i = 0; i < 3; ++i)
		covariance(i, i) = 0.01;
	for (int i = 3; i < 6; ++i)
		covariance(i, i) = 0.004;

	//const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
	if (!loop_closure_.AddBetweenFactor(localization_.GetIncrementalEstimate(),
		covariance,&pose_key)) {
		return false;
	}
	*new_keyframe = true;

	if (!loop_closure_.AddKeyScanPair(pose_key, scan)) {
		return false;
	}




	std::vector<unsigned int> closure_keys;
	if (!loop_closure_.FindLoopClosures(pose_key, &closure_keys)) {
		return false;
	}

	for (const auto& closure_key : closure_keys) {
		printf(": Closed loop between poses %u and %u.\n", 
			pose_key, closure_key);
	}
	return true;
}


void BlamSlam::ProcessPointCloudMessage(const PointCloud::ConstPtr& msg) {
	PointCloud::Ptr msg_filtered(new PointCloud);
	filter_.Filter(msg, msg_filtered);
	if (!odometry_.UpdateEstimate(*msg_filtered)) {
		PointCloud::Ptr unused(new PointCloud);
		mapper_.InsertPoints(msg_filtered, unused.get());
		loop_closure_.AddKeyScanPair(0, msg);
		return;
	}
	PointCloud::Ptr msg_transformed(new PointCloud);
	PointCloud::Ptr msg_neighbors(new PointCloud);
	PointCloud::Ptr msg_base(new PointCloud);
	PointCloud::Ptr msg_fixed(new PointCloud);
	localization_.MotionUpdate(odometry_.GetIncrementalEstimate());
	localization_.TransformPointsToFixedFrame(*msg_filtered,
		msg_transformed.get());
	mapper_.ApproxNearestNeighbors(*msg_transformed, msg_neighbors.get());
	localization_.TransformPointsToSensorFrame(*msg_neighbors, msg_neighbors.get());
	localization_.MeasurementUpdate(msg_filtered, msg_neighbors, msg_base.get());

	bool new_keyframe;
	if (HandleLoopClosures(msg, &new_keyframe)) {
		printf("have found Loop Closures!\n");
		PointCloud::Ptr regenerated_map(new PointCloud);
		loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

		mapper_.Reset();
		PointCloud::Ptr unused(new PointCloud);
		mapper_.InsertPoints(regenerated_map, unused.get());

		localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
	}
	else {
		if (new_keyframe) {
			localization_.MotionUpdate(gu::Transform3::Identity());
			localization_.TransformPointsToFixedFrame(*msg, msg_fixed.get());
			PointCloud::Ptr unused(new PointCloud);
			mapper_.InsertPoints(msg_fixed, unused.get());
		}
	}
	//PointCloud::Ptr regenerated_map(new PointCloud);
	//loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());
	//mapper_.Reset();
	//PointCloud::Ptr unused(new PointCloud);
	//mapper_.InsertPoints(regenerated_map, unused.get());


}
std::string BlamSlam::itos(int i)   // ½«int ×ª»»³Éstring 
{
	std::stringstream s;
	s << i;
	return s.str();
}
bool BlamSlam::showPointCloud(int FrameCounter)
{
	//const gu::Transform3 estimate = localization_.GetIntegratedEstimate();
	//const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
	//const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();
	//Eigen::Matrix4d ICP_Transform;
	//ICP_Transform.block(0, 0, 3, 3) = R;
	//ICP_Transform.block(0, 3, 3, 1) = T;
	//const Eigen::Affine3d tf;
	//Eigen::Transform<double, 3, Eigen::Affine> t(ICP_Transform);
	//if (FrameCounter == FIRST_FRAME)
	//{
	//	viewer_.addCoordinateSystem(1.0, (const Eigen::Affine3f)tf, "reference", 0);     //Add CoordinateSystem
	//}
	//viewer_.addPointCloud<pcl::POINT_TYPE>(mapper_.map_data_, itos(FrameCounter));
	//viewer_.updateCoordinateSystemPose("reference", (const Eigen::Affine3f)tf); //update CoordinateSystem
	//viewer_.spinOnce(10);
	return 0;
}