#include <blam_slam/BlamSlam.h>
#include"common.h"
#include"hdevicePose.h"

#include <point_cloud_read/pcap_read.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
// 类型定义
#include <geometry_utils/Transform3.h>
//#include <parameter_utils/ParameterUtils.h>
#include<sstream>
#include <pcl/filters/passthrough.h>
//namespace pu = parameter_utils;
namespace gu = geometry_utils;

typedef pcl::POINT_TYPE PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace pcl::visualization;
BlamSlam blamSlam_;

PointCloud::Ptr pcap2PointCloud(std::vector<_array> &Points)
{
	PointCloud::Ptr cloud(new PointCloud);
	int cnt = 0;
	for (int i = 0; i < Points.size(); i++)
	{
		//if (Points[i].LaserId == 1)
		{
			PointT p;
			cnt++;
			p.z = Points[i].xyz[2];
			p.x = Points[i].xyz[0];
			p.y = Points[i].xyz[1];

			p.intensity = Points[i].xyz[2];
			cloud->points.push_back(p);
		}
	
	}
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cnt;
	cloud->is_dense = false;

	return cloud;
}
std::string itos(int i)   // 将int 转换成string 
{
	std::stringstream s;
	s << i;
	return s.str();
}

long long frameNumber = 0;
bool endFrame = 0;
pcl::PointXYZ pointCam_old, pointCam_new;
Eigen::Transform<double, 3, Eigen::Affine> ICP_Transform;
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	if (endFrame)
	{
		return;
	}
	static long long cnt = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << frameNumber;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 50, 50, "text", 0);
	if (frameNumber > 1)
	{
		//viewer.addLine(pointCam_old, pointCam_new, 1, 0, 0, itos(cnt++), 0);
	}
	if (frameNumber == 1)
	{
		viewer.addCoordinateSystem(1.0, (Eigen::Affine3f)ICP_Transform, "reference", 0);     //Add CoordinateSystem
	}
	viewer.updateCoordinateSystemPose("reference", (Eigen::Affine3f)ICP_Transform);
	//viewer.updateCoordinateSystemPose("reference", (Eigen::Affine3f)ICP_Transform); //update CoordinateSystem
	//FIXME: possible race condition here:  
}

//DWORD WINAPI VisualizationThread(LPVOID lpparentet)
//{
//	pcl::visualization::PCLVisualizer viewer("viewer1");
//	viewer.addCoordinateSystem(1.0, (Eigen::Affine3f)ICP_Transform, "reference", 0);     //Add CoordinateSystem
//	static long long cnt = 0;
//
//	while (!viewer.wasStopped())
//	{
//		viewer.addPointCloud<pcl::POINT_TYPE>(blamSlam_.mapper_.map_data_, itos(frameNumber++));
//		const gu::Transform3 estimate = blamSlam_.localization_.GetIntegratedEstimate();
//		const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
//		const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();
//		Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();;
//		tf.block(0, 0, 3, 3) = R;
//		tf.block(0, 3, 3, 1) = T;
//		pcl::PointXYZ  mm;
//		mm.x = T(0);
//		mm.y = T(1);
//		mm.z = T(2);
//		pointCam_old = pointCam_new;
//		pointCam_new = mm;
//		ICP_Transform = Eigen::Transform<double, 3, Eigen::Affine>(tf);
//		std::stringstream ss;
//		ss << "Once per viewer loop: " << frameNumber;
//		viewer.removeShape("text", 0);
//		viewer.addText(ss.str(), 50, 50, "text", 0);
//		viewer.addLine(pointCam_old, pointCam_new, 1, 0, 0, itos(cnt++), 0);
//		viewer.updateCoordinateSystemPose("reference", (Eigen::Affine3f)ICP_Transform);
//		viewer.spinOnce(10);
//	}
//	
//	return 0;
//}

int main(void)
{
//	pcl::visualization::CloudViewer viewer("viewer2");
//
//	PointCloud::Ptr cloud_(new PointCloud);
//	pcl::io::loadPCDFile("./2.pcd", *cloud_);
//
////	pcl::PassThrough<pcl::POINT_TYPE> pass;
////	pcl::PointCloud<pcl::POINT_TYPE>::Ptr cloud_(new pcl::PointCloud<pcl::POINT_TYPE>);
//	/*pass.setInputCloud(cloud_);
//	pass.setFilterFieldName("z");
//	pass.setFilterLimits(0.0, 1.0);
//	pass.filter(*cloud_filtered);*/
//	int maxDAT = 0;
//	int minDAT = 10000;
//
//	for (int i = 0; i < cloud_->size(); i++)
//	{
//		if (maxDAT < cloud_->points[i].z)
//		{
//			maxDAT = (int)cloud_->points[i].z;
//		}
//		if (minDAT > cloud_->points[i].z)
//		{
//			minDAT = (int)cloud_->points[i].z;
//		}
//	}
//	std::cout << "max: " << maxDAT << "min: " << minDAT << std::endl;
//	float t = (maxDAT - minDAT + 0.00001) / 10.0;
//	for (int i = 0; i < cloud_->size(); i++)
//	{
//		cloud_->points[i].intensity = ((int)((cloud_->points[i].z - minDAT) / t+0.5)) * 20;
//	}
//	viewer.showCloud(cloud_, "viewer2");
//	while (!viewer.wasStopped()){}
//
	const std::string fileNamePcap = "E:\\work-nd\\data\\160921-150645\\lidar.pcap";
	const std::string calibrationPath = "./VLP-16.xml";
	MyHdlGrabber _myHdlGrabber(fileNamePcap, calibrationPath);

	blamSlam_.Initialize();
	pcl::visualization::CloudViewer viewer2("viewer1");
	viewer2.runOnVisualizationThread(viewerPsycho);

//	CreateThread(NULL, 0, VisualizationThread, NULL, 0, NULL);
	devicePose devicePose_;
	FILE* file = fopen("corRT.txt", "w");

	static int cnt = FIRST_FRAME;
	while (!viewer2.wasStopped())
	{
		if (endFrame)
		{
			continue;
		}
		if (cnt > 1580)
		{
			std::cout << "No Frame!" << std::endl;
			endFrame = 1;
			pcl::PCDWriter writer;
			writer.write("./1.pcd", *blamSlam_.mapper_.map_data_);
			continue;
		}
		if (_myHdlGrabber.GetFrame(cnt++)) {
			fclose(file);
			std::cout << "No Frame!" << std::endl;
			endFrame = 1;
			pcl::PCDWriter writer;
			writer.write("./save_c_loop.pcd", *blamSlam_.mapper_.map_data_);
			continue;
		}
		PointCloud::Ptr newCloud = pcap2PointCloud(_myHdlGrabber.Points);
		pcl::PCDWriter writer;
		writer.write("./1.pcd", *newCloud);
		blamSlam_.ProcessPointCloudMessage(newCloud);
		const gu::Transform3 estimate = blamSlam_.localization_.GetIntegratedEstimate();
		const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
		const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();
		Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();;
		tf.block(0, 0, 3, 3) = R;
		tf.block(0, 3, 3, 1) = T;
		pcl::PointXYZ  mm;
		mm.x = T(0);
		mm.y = T(1);
		mm.z = T(2);
		double szR[9];
		double szT[3];
		

		for (int x = 0; x < 3; x++)
		{
			szT[x] = T(x);
			for (int y = 0; y < 3; y++)
			{
				szR[x * 3 + y] = R(x * 3 + y);
			}
		}

		devicePose_.setR(szR);
		devicePose_.setT(szT);
		devicePose_.write(file);
		//fprintf(file, "\r\n");
		//fclose(file);

		pointCam_old = pointCam_new;
		pointCam_new = mm;
		ICP_Transform = Eigen::Transform<double, 3, Eigen::Affine>(tf);
		viewer2.showCloud(blamSlam_.mapper_.map_data_, "viewer1");
		frameNumber++;
		//showPointCloud(cnt - 1, newCloud);
	}
 	return 0;
}
