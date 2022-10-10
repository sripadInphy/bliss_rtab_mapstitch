#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/ParamEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include "rtabmap/utilite/UStl.h"
#include <rtabmap/utilite/UTimer.h>


#include <bliss/MapData.h>
#include "bliss/MsgConversion.h"
using namespace std;
int map_count = 0;
int map_no = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalcloud(new pcl::PointCloud<pcl::PointXYZRGB>);


void mapDataCallback( const bliss::MapDataConstPtr& msg)
{
	const bliss::MapData& map = *msg;
	
	cout<<map_count<<endl;
	if(map_count > 5){
		// MapData
		rtabmap::Transform mapToOdom;
		std::map<int, rtabmap::Transform> poses;
		std::map<int, rtabmap::Signature> signatures;
		std::multimap<int, rtabmap::Link> links;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(unsigned int i=0; i<map.graph.posesId.size() && i<map.graph.poses.size(); ++i)
		{
			poses.insert(std::make_pair(map.graph.posesId[i], bliss::transformFromPoseMsg(map.graph.poses[i])));
		}
		std::set<int> nodeDataReceived;
		for(unsigned int i=0; i<map.nodes.size() && i<map.nodes.size(); ++i)
		{
			int id = map.nodes[i].id;

			rtabmap::Signature s = bliss::nodeDataFromROS(map.nodes[i]);
			s.sensorData().uncompressData();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp;
			pcl::IndicesPtr validIndices(new std::vector<int>);

			tmp = rtabmap::util3d::cloudRGBFromSensorData(
					s.sensorData(),
					4,
					4.0f,
					0.0f,
					validIndices.get());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::vector<int> index;
			pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
			if(!tmpNoNaN->empty())
			{
				*cloud += *rtabmap::util3d::transformPointCloud(tmpNoNaN, poses.find(id)->second); // transform the point cloud to its pose
			}                        
		}
		if(cloud->size())
		{
			printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
			cloud = rtabmap::util3d::voxelize(cloud, 0.01f);
			*globalcloud += *cloud; 
			//Post processing of point clouds
			//SMoothing/////////////////////
			// pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			// pcl::PointCloud<pcl::PointNormal> mls_points;
			// pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
			// mls.setInputCloud (globalcloud);
			// mls.setPolynomialOrder (2);
			// mls.setSearchMethod (tree);
			// mls.setSearchRadius (0.03);
			// mls.process (mls_points);  

			//Fast triangulation Method ///////////////////
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			tree->setInputCloud (globalcloud);
			n.setInputCloud (globalcloud);
			n.setSearchMethod (tree);
			n.setKSearch (20);
			n.compute (*normals);
			// Concatenate the XYZ and normal fields*
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
			//* cloud_with_normals = cloud + normals


			printf("Saving rtabmap_cloud.ply... done! (%d points)\n", (int)cloud->size());
			pcl::io::savePLYFile("/home/inphys/Desktop/pointcclouds/blissfulclouds/rtabmap_cloud"+to_string(map_no)+".ply", *cloud); // to save in PLY format
			pcl::io::savePLYFile("/home/inphys/Desktop/pointcclouds/blissfulclouds/global_cloud"+to_string(map_no)+".ply", *globalcloud);  //Concat to make global cloud
			// pcl::io::savePLYFile ("/home/inphys/Desktop/pointcclouds/blissfulclouds/smooth_global_cloud"+to_string(map_no)+".ply", mls_points);
			pcl::io::savePLYFile ("/home/inphys/Desktop/pointcclouds/blissfulclouds/smooth_global_cloud"+to_string(map_no)+".ply", mls_points);
			map_no++;
		}
		else
		{
			printf("Saving rtabmap_cloud.pcd... failed! The cloud is empty.\n");
		}
		
		map_count = 0; 	
	}
	else map_count++;
	
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "blissfullStitchin");
	ros::NodeHandle n;
	std::cout << "Test subscribe";
	ros::Subscriber sub = n.subscribe("/rtabmap/mapData", 1, mapDataCallback);
  	ros::spin();

  	return 0;
}