#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

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

void mapDataCallback( const bliss::MapDataConstPtr& msg)
{
	cout<<map_count<<endl;
	if(map_count > 10){
		// MapData
	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::map<int, rtabmap::Signature> signatures;
	std::multimap<int, rtabmap::Link> links;


	cout << "Callback called"<<endl;

	bliss::mapDataFromROS(*msg, poses, links, signatures, mapToOdom);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		
		rtabmap::Signature signode = signatures.find(iter->first)->second;
		cout<<"got passed"<<endl;
		// signode.sensorData().uncompressData();
		// cout<<"sensorData was above this"<<endl;
		// pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
		// 		signode.sensorData(),
		// 		4,           // image decimation before creating the clouds
		// 		4.0f,        // maximum depth of the cloud
		// 		0.0f);
		// pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
		// std::vector<int> index;
		// pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
		// cout<<"RemoveNAN worked thenm"<<endl;
		// if(!tmpNoNaN->empty())
		// {
		// 	*cloud += *rtabmap::util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
		// }
	}
	if(cloud->size())
	{
		printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
		cloud = rtabmap::util3d::voxelize(cloud, 0.01f);

		printf("Saving rtabmap_cloud.pcd... done! (%d points)\n", (int)cloud->size());
		// pcl::io::savePCDFile("rtabmap_cloud.pcd", *cloud);
		pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY format
	}
	else
	{
		printf("Saving rtabmap_cloud.pcd... failed! The cloud is empty.\n");
	}
	// map_count = 0;
	}
	else map_count++;
	

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "blissfullStitchin");
	ros::NodeHandle n;
	std::cout << "Test subscribe";
	ros::Subscriber sub = n.subscribe("/rtabmap/mapData", 10, mapDataCallback);
  	ros::spin();

  	return 0;
}