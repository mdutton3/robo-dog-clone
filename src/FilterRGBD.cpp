/*------------------------------------------------------------------*/

#include <sstream>
#include <cstdio>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <ach.h>
#include <ctime>
#include <amino.hpp>
#include <amino/time.h>

#include "RGB2HSV.hpp"

using pcl::PointXYZ;
using pcl::PointXYZRGB;
using pcl::PointCloud;

typedef PointCloud<PointXYZ> PointCloudXYZ;
typedef PointCloud<PointXYZRGB> PointCloudRGB;

struct HSV
{
	inline HSV() :
		h(0), s(0), v(0)
	{
	}
	inline HSV(uint8_t h_, uint8_t s_, uint8_t v_) :
		h(h_), s(s_), v(v_)
	{
	}
	inline HSV(HSV const & rhs) :
		h(rhs.h), s(rhs.s), v(rhs.v)
	{
	}

	uint8_t h;
	uint8_t s;
	uint8_t v;
};

class FilterRGBD
{
	ros::NodeHandle nh_;
	ros::Subscriber pc_sub;
	ros::Publisher pc_pub;
	ros::Publisher centroid_pub;
	HSV hsv_min;
	HSV hsv_max;

	Eigen::Vector4f centroid_filtered;

	std::vector<size_t> indices;
	PointCloudXYZ::Ptr cloud_filtered;
	PointCloudRGB cluster_pc;

	std::string ach_channel_name;
	ach_channel_t ach_channel;
	bool ach_valid;

	int cycle_count;
	timespec last_update;

public:
	FilterRGBD()
		: nh_("~")
		//, hsv_min(20, 110, 110), hsv_max(45, 255, 255)
		, hsv_min(60, 110, 110), hsv_max(80, 255, 255)
		, centroid_filtered(0,0,0,0)
		, cloud_filtered(new PointCloudXYZ)
		, ach_channel_name("obj-centroid"), ach_channel(), ach_valid(false)
		, cycle_count(0)
	{
		pc_sub = nh_.subscribe("in", 1, &FilterRGBD::OnPC, this);
		pc_pub = nh_.advertise<PointCloudRGB> ("out", 10);
		centroid_pub = nh_.advertise<PointCloudXYZ> ("centroid", 10);

		int temp;
		if (nh_.getParam("hue_min", temp))
			hsv_min.h = temp;
		if (nh_.getParam("sat_min", temp))
			hsv_min.s = temp;
		if (nh_.getParam("val_min", temp))
			hsv_min.v = temp;

		if (nh_.getParam("hue_max", temp))
			hsv_max.h = temp;
		if (nh_.getParam("sat_max", temp))
			hsv_max.s = temp;
		if (nh_.getParam("val_max", temp))
			hsv_max.v = temp;

		nh_.getParam("ach_channel", ach_channel_name);

		int r = ach_open(&ach_channel, ach_channel_name.c_str(), NULL);
		ach_valid = (ACH_OK == r);
		if( !ach_valid )
		{
			ROS_ERROR( "Could not open ACH channel[%s], error: %d", ach_channel_name.c_str(), r );
		}

		last_update = aa_tm_now();
	}

	~FilterRGBD()
	{
	}

	void OnPC(PointCloudRGB const & pc)
	{
		indices.clear();
		cloud_filtered->points.clear();

		for (size_t i = 0; i < pc.points.size(); ++i)
		{
			PointXYZRGB const & pt = pc.points[i];
			HSV hsv;
			convertRGBtoHSV(pt, hsv);

			if (!(pt.x == pt.x))
				continue;

			if ((hsv.h < hsv_min.h) || (hsv.h > hsv_max.h))
				continue;
			if ((hsv.s < hsv_min.s) || (hsv.s > hsv_max.s))
				continue;
			if ((hsv.v < hsv_min.v) || (hsv.v > hsv_max.v))
				continue;

			cloud_filtered->push_back(PointXYZ(pt.x, pt.y, pt.z));
			indices.push_back(i);
		}

		typedef pcl::EuclideanClusterExtraction<PointXYZ> ECE;
		typedef pcl::KdTreeFLANN<PointXYZ> KdTree;
		KdTree::Ptr tree(new KdTree);
		tree->setInputCloud(cloud_filtered);

		ECE ec;
		ec.setClusterTolerance(0.01);
		ec.setMinClusterSize(25);
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

		cluster_pc.points.reserve(cluster_idx.indices.size());
		for (size_t i = 0; i < cluster_idx.indices.size(); ++i)
		{
			size_t filtered_idx = cluster_idx.indices[i];
			size_t idx = indices[filtered_idx];
			cluster_pc.points.push_back(pc.points[idx]);
		}

		cluster_pc.header = pc.header;
		cluster_pc.height = cluster_pc.points.size();
		cluster_pc.width = 1;
		cluster_pc.is_dense = true;
		cluster_pc.sensor_orientation_ = pc.sensor_orientation_;
		cluster_pc.sensor_origin_ = pc.sensor_origin_;

		pc_pub.publish( cluster_pc );

		Eigen::Vector4f centroid_sample;
		pcl::compute3DCentroid(cluster_pc, centroid_sample);

		static double const sample_weight = 0.80;
		centroid_filtered = centroid_filtered * (1.0 - sample_weight)
				+ centroid_sample * sample_weight;

		PointCloudXYZ centroid_pc;
		centroid_pc.push_back(pcl::PointXYZ(centroid_filtered[0],
				centroid_filtered[1], centroid_filtered[2]));
		centroid_pc.header = pc.header;
		centroid_pc.height = centroid_pc.width = 1;
		centroid_pc.is_dense = true;
		centroid_pc.sensor_orientation_ = pc.sensor_orientation_;
		centroid_pc.sensor_origin_ = pc.sensor_origin_;
		centroid_pub.publish(centroid_pc);

		publishToAch(centroid_filtered);

		++cycle_count;
		if ((cycle_count % 100) == 0)
		{
			cycle_count = 0;
			timespec t_now = aa_tm_now();
			double delta = aa_tm_timespec2sec(aa_tm_sub(t_now, last_update));
			fprintf(stderr,
					"\t100 cycles in %lf seconds for nominal rate of %lf\n",
					delta, 100.0 / delta);
			last_update = t_now;
		}
	}

	void publishToAch(Eigen::Vector4f const & pt)
	{
		if (ach_valid)
		{
			double data[3] = { pt[0], pt[1], pt[2] };
			ach_put(&ach_channel, data, sizeof(data));
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "FilterRGBD");
	FilterRGBD node;
	ros::spin();
	return 0;
}

