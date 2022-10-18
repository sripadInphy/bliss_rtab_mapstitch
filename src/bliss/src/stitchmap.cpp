#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>



#include <pcl/make_shared.h>  // for pcl::make_shared
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr prevcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>),source_map,target_map;
Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
// void PointCloud2Vector3d (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
// {
//   for (unsigned i = 0; i < cloud->size (); i++)
//   {
//     Point &p = cloud->at (i);
// 	   if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
//       data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
//   }
// }



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
					1.0f,
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
			cloud = rtabmap::util3d::voxelize(cloud, 0.001f);
			*globalcloud += *cloud; 
			//Post processing of point clouds
			//SMoothing############################################
			// pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			// pcl::PointCloud<pcl::PointNormal> mls_points;
			// pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
			// mls.setInputCloud (globalcloud);
			// mls.setPolynomialOrder (2);
			// mls.setSearchMethod (tree);
			// mls.setSearchRadius (0.03);
			// mls.process (mls_points);  
			//######################################################


			//Fast triangulation Method ############################
			// pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
			// pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			// pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			// tree->setInputCloud (cloud);
			// n.setInputCloud (cloud);
			// n.setSearchMethod (tree);
			// n.setKSearch (20);
			// n.compute (*normals);
			// // Concatenate the XYZ and normal fields*
			// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
			// pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

			// // Create search tree*
		    // pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  			// tree2->setInputCloud (cloud_with_normals);

			// // Initialize objects
			// pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
			// pcl::PolygonMesh triangles;

  			// // Set the maximum distance between connected points (maximum edge length)
	    	// gp3.setSearchRadius (0.025);


			// // Set typical values for the parameters
			// gp3.setMu (2.5);
			// gp3.setMaximumNearestNeighbors (100);
			// gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			// gp3.setMinimumAngle(M_PI/18); // 10 degrees
			// gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			// gp3.setNormalConsistency(false);

			// // Get result
  			// gp3.setInputCloud (cloud_with_normals);
  			// gp3.setSearchMethod (tree2);
  			// gp3.reconstruct (triangles);

			// //#######################################################

			// B spline surface reconstruction

			//write in ply format
			printf("Saving rtabmap_cloud.ply... done! (%d points)\n", (int)cloud->size());
			// pcl::io::savePLYFile("/home/inphys/Desktop/pointcclouds/blissfulclouds/rtabmap_cloud"+to_string(map_no)+".ply", *cloud); // to save in PLY format
			// pcl::io::savePLYFile("/home/inphys/Desktop/pointcclouds/blissfulclouds/global_cloud"+to_string(map_no)+".ply", *globalcloud);  //Concat to make global cloud
			// pcl::io::savePLYFile ("/home/inphys/Desktop/pointcclouds/blissfulclouds/smooth_global_cloud"+to_string(map_no)+".ply", mls_points);
			// pcl::io::savePLYFile ("/home/inphys/Desktop/pointcclouds/blissfulclouds/smooth_global_cloud"+to_string(map_no)+".ply", triangles);
			
			
			//write in pcd format 
			// pcl::io::savePCDFile ("/home/inphys/Desktop/pointcclouds/blissfulclouds/pcd/global_cloud"+to_string(map_no)+".pcd", *globalcloud);
			 pcl::io::savePCDFileASCII ("/home/inphys/Desktop/pointcclouds/blissfulclouds/pcd/global_cloud"+to_string(map_no)+".pcd", *globalcloud);
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


// class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
// {
//   using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
// public:
//   MyPointRepresentation ()
//   {
//     // Define the number of dimensions
//     nr_dimensions_ = 4;
//   }

//   // Override the copyToFloatArray method to define our feature vector
//   virtual void copyToFloatArray (const PointNormalT &p, float * out) const
//   {
//     // < x, y, z, curvature >
//     out[0] = p.x;
//     out[1] = p.y;
//     out[2] = p.z;
//     out[3] = p.curvature;
//   }
// };



// void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
// {
//   //
//   // Downsample for consistency and speed
//   // \note enable this for large datasets
//   PointCloud::Ptr src (new PointCloud);
//   PointCloud::Ptr tgt (new PointCloud);
//   pcl::VoxelGrid<PointT> grid;
//   if (downsample)
//   {
//     grid.setLeafSize (0.05, 0.05, 0.05);
//     grid.setInputCloud (cloud_src);
//     grid.filter (*src);

//     grid.setInputCloud (cloud_tgt);
//     grid.filter (*tgt);
//   }
//   else
//   {
//     src = cloud_src;
//     tgt = cloud_tgt;
//   }


//   // Compute surface normals and curvature
//   PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
//   PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

//   pcl::NormalEstimation<PointT, PointNormalT> norm_est;
//   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//   norm_est.setSearchMethod (tree);
//   norm_est.setKSearch (30);
  
//   norm_est.setInputCloud (src);
//   norm_est.compute (*points_with_normals_src);
//   pcl::copyPointCloud (*src, *points_with_normals_src);

//   norm_est.setInputCloud (tgt);
//   norm_est.compute (*points_with_normals_tgt);
//   pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

//   //
//   // Instantiate our custom point representation (defined above) ...
//   MyPointRepresentation point_representation;
//   // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
//   float alpha[4] = {1.0, 1.0, 1.0, 1.0};
//   point_representation.setRescaleValues (alpha);

//   //
//   // Align
//   pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
//   reg.setTransformationEpsilon (1e-6);
//   // Set the maximum distance between two correspondences (src<->tgt) to 10cm
//   // Note: adjust this based on the size of your datasets
//   reg.setMaxCorrespondenceDistance (0.1);  
//   // Set the point representation
//   reg.setPointRepresentation (pcl::make_shared<const MyPointRepresentation> (point_representation));

//   reg.setInputSource (points_with_normals_src);
//   reg.setInputTarget (points_with_normals_tgt);



//   //
//   // Run the same optimization in a loop and visualize the results
//   Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
//   PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
//   reg.setMaximumIterations (2);
//   for (int i = 0; i < 30; ++i)
//   {
//     PCL_INFO ("Iteration Nr. %d.\n", i);

//     // save cloud for visualization purpose
//     points_with_normals_src = reg_result;

//     // Estimate
//     reg.setInputSource (points_with_normals_src);
//     reg.align (*reg_result);

// 		//accumulate transformation between each Iteration
//     Ti = reg.getFinalTransformation () * Ti;

// 		//if the difference between this transformation and the previous one
// 		//is smaller than the threshold, refine the process by reducing
// 		//the maximal correspondence distance
//     if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
//       reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
//     prev = reg.getLastIncrementalTransformation ();

 
//   }

// 	//
//   // Get the transformation from target to source
//   targetToSource = Ti.inverse();

//   //
//   // Transform target back in source frame
//   pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

//   //add the source to the transformed target
//   *output += *cloud_src;
  
//   final_transform = targetToSource;
//  }


// void mapcallback(const sensor_msgs::PointCloud2ConstPtr& cloud ){
// 	pcl::PCLPointCloud2 pcl_pc2;
// 	pcl_conversions::toPCL(*cloud,pcl_pc2);
// 	if(map_count++ > 1){
// 		source_map = result;
// 		// target_map = *cloud;
// 		pcl::fromPCLPointCloud2(pcl_pc2,*target_map);

// 		PointCloud::Ptr temp (new PointCloud);
		
// 		pairAlign (source_map, target_map, temp, pairTransform, true);

// 		//transform current pair into the global transform
// 		pcl::transformPointCloud (*temp, *result, GlobalTransform);

// 		//update the global transform
// 		GlobalTransform *= pairTransform;

// 		//save aligned pair, transformed into the first cloud's frame
// 		std::stringstream ss;
// 		ss << map_no++ << ".pcd";
// 		pcl::io::savePCDFile (ss.str (), *result, true);
// 		std::cout << "Map Saved"<<endl;
// 	}else {
// 		pcl::fromPCLPointCloud2(pcl_pc2,*result);
// 	}
	
	
// }


int main(int argc, char **argv)
{

	ros::init(argc, argv, "blissfullStitchin");
	ros::NodeHandle n;
	std::cout << "Test subscribe";
	ros::Subscriber sub = n.subscribe("/rtabmap/mapData", 1, mapDataCallback);
  	ros::spin();

  	return 0;
}