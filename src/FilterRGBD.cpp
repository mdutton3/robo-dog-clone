/*------------------------------------------------------------------*/

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <cstdio>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include "RGB2HSV.hpp"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;


struct HSV
{
	uint8_t h;
	uint8_t s;
	uint8_t v;
};

class FilterRGBD
{
	ros::NodeHandle nh_;
	ros::Subscriber pc_sub;
	ros::Publisher pc_pub;
	CvScalar const ball_hsv_min;
	CvScalar const ball_hsv_max;

public:
	FilterRGBD() :
		nh_("~"), ball_hsv_min(cvScalar(20, 110, 110, 0)), ball_hsv_max(
				cvScalar(45, 256, 256, 0))
	{
		pc_sub = nh_.subscribe("in", 1, &FilterRGBD::OnPC, this);
		pc_pub = nh_.advertise<PointCloud> ("out", 10);
	}

	~FilterRGBD()
	{
	}

	void OnPC(PointCloud const & pc)
	{

		std::vector<size_t> indices;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<
				pcl::PointXYZ>);
		cloud_filtered->points.reserve(pc.points.size() / 4);

		for (size_t i = 0; i < pc.points.size(); ++i)
		{
			PointType const & pt = pc.points[i];
			struct HSV hsv;
			convertRGBtoHSV(pt, hsv);

			if (!(pt.x == pt.x))
				continue;

			if ((hsv.h < ball_hsv_min.val[0]) || (hsv.h > ball_hsv_max.val[0]))
				continue;
			if ((hsv.s < ball_hsv_min.val[1]) || (hsv.s > ball_hsv_max.val[1]))
				continue;
			if ((hsv.v < ball_hsv_min.val[2]) || (hsv.v > ball_hsv_max.val[2]))
				continue;

			cloud_filtered->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
			indices.push_back(i);
		}

		typedef pcl::EuclideanClusterExtraction<pcl::PointXYZ> ECE;
		typedef pcl::KdTreeFLANN<pcl::PointXYZ> KdTree;
		KdTree::Ptr tree(new KdTree);
		tree->setInputCloud(cloud_filtered);

		ECE ec;
		ec.setClusterTolerance(0.02);
		ec.setMinClusterSize(100);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		ec.extract(cluster_indices);

		size_t iBiggest = 0;
		for (size_t i = 1; i < cluster_indices.size(); ++i)
		{
			if (cluster_indices[i].indices.size()
					> cluster_indices[iBiggest].indices.size())
				iBiggest = i;
		}

		if (iBiggest >= cluster_indices.size())
			return;

		pcl::PointIndices const & cluster_idx = cluster_indices[iBiggest];
		PointCloud out;
		out.points.reserve(cluster_idx.indices.size());
		for (size_t i = 0; i < cluster_idx.indices.size(); ++i)
		{
			size_t filtered_idx = cluster_idx.indices[i];
			size_t idx = indices[filtered_idx];
			out.points.push_back(pc.points[idx]);
		}

		out.header = pc.header;
		out.height = out.points.size();
		out.width = 1;
		out.is_dense = true;
		out.sensor_orientation_ = pc.sensor_orientation_;
		out.sensor_origin_ = pc.sensor_origin_;

		pc_pub.publish(out);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "FilterRGBD");
	FilterRGBD node;
	ros::spin();
	return 0;
}

