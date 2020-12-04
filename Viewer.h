#pragma once

#include <iostream>
#include <fstream>
#include <assert.h>     /* assert */
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/console/parse.h>
#include <Math.h>

#include "custom_point.hpp"
#include "OccupancyGrid.h"
#include "VariableOccupancyGrid.h"
#include "Grid_Settings.h"
#include "Common.h"
#include "BinFileReader.h"
#include "CsvFileReader.h"

#include <chrono>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;


class SystemViewer
{
private:
	bool file_reader_csv;
public:
	typedef pcl::PointCloud<PointCustom> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	/*
	* file_reader_csv = true - means the file with the cloud is in csv format, else bin format
	*/
	SystemViewer(
		pcl::visualization::PointCloudColorHandler<PointCustom> &handler) :
		cloud_viewer_(new pcl::visualization::PCLVisualizer("3D-Viewer")),
		handler_(handler), bool file_reader_csv=true
	{
		this->file_reader_csv = file_reader_csv;
	}

#pragma region runnable

	void run()
	{
		cloud_viewer_->setBackgroundColor(0.1647, 0.2313, 0.1529);
		cloud_viewer_->addCube(-2, 1, -0.8, 0.8, 0, 1.6);
		cloud_viewer_->addCoordinateSystem(1, 0);
		pcl::PointXYZ p0, p1;
		p0.x = 0;
		p0.y = 0;
		p0.z = 1.6;
		p1.x = 1;
		p1.y = 0;
		p1.z = 1.6;
		
		cloud_viewer_->addArrow<PointXYZ, PointXYZ>(p1, p0, 1.0, 0.0, 0.0, false, "arrow", 0);
		cloud_viewer_->initCameraParameters();
		cloud_viewer_->setCameraPosition(0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0);
		cloud_viewer_->setCameraClipDistances(0.0, 10.0);

		cloud_viewer_->registerKeyboardCallback(keyboardEventOccurred);


		Cloud myCloud;


		vector<vector<string>> cloudSequencesLocation(NR_VELOS);

		vector<string> egoInfoLocation = vector<string>();






		int maxLidarRange = MAX_LIDAR_DEPTH_RANGE / 4;
		OccupancyGrid *occup_grid = new OccupancyGrid(GRID_RESOLUTION, (-1)*maxLidarRange, (-1)*maxLidarRange, maxLidarRange, maxLidarRange);
		//GridMap *grid_depth = new GridMap(3,0,0,camera[0].cols,camera[0].rows);
		//MVGM *grid = new MVGM(GRID_RESOLUTION, (-1)*30, (-1)*40, -0.3,50, 40,4);
		//MVGM *grid = new MVGM(GRID_RESOLUTION, (-1) * maxLidarRange, (-1) * maxLidarRange, -0.3, maxLidarRange, maxLidarRange, 4);

		myCloud = Cloud();

		FrontCamProjections camFrontProjection = FrontCamProjections();

		
		string file_path = ""; //include file path csv/bin
		if(file_reader_csv)
			ReadCSVFiles r = ReadCSVFiles();
			r.getCloudVeloFromCSV(file_path,myCloud);
		else
			BinFileReader r = BinFileReader();
			r.getCloudVeloFromBin(file_path,myCloud);
			
		int visualization_type=0;//default - show the cloud

		string vlp_could_name = "Cloud0"; //random name

		cv::waitKey();

		while (true) {
	
			auto t1 = std::chrono::high_resolution_clock::now();
			switch (visualization_type) {
			case 0:
				Visualization_OriginalCloud(cloud_viewer_, handler_, myCloud, vlp_could_name);
				break;
			case 1: //press g
				//view occupancy grid
				Visualization_OccupancyGridCloud(cloud_viewer_, handler_, myCloud, vlp_could_name,occup_grid);
				break;
			case 2: //press s
				//view elevation map  grid
				Visualization_ElevationGridCloud(cloud_viewer_, handler_, myCloud, vlp_could_name,occup_grid);
				break;
			case 3: //press q

				break;
			default:
				break;
			}

			auto t2 = std::chrono::high_resolution_clock::now();
			std::chrono::duration<int64_t, std::nano> elapsed = t2 - t1;
			
			cloud_viewer_->spinOnce();
			
			cloud_viewer_->removePointCloud(vlp_could_name);
			cloud_viewer_->removeAllShapes();
			boost::this_thread::sleep(boost::posix_time::microseconds(100));

		}
		


	}


#pragma endregion


	boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

	boost::mutex cloud_mutex_;

	pcl::visualization::PointCloudColorHandler<PointCustom> &handler_;


	void Visualization_OriginalCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viz, pcl::visualization::PointCloudColorHandler<PointCustom> &handler, PointCloud<PointCustom> &P, string PCL_name) {

		viz->spinOnce();
		viz->removePointCloud(PCL_name);

		CloudConstPtr cloud;
		cloud = P.makeShared();
		handler.setInputCloud(cloud);
		if (!viz->updatePointCloud(cloud, handler, PCL_name)) {
			viz->addPointCloud(cloud, handler, PCL_name);
		}
		P.clear();

		boost::this_thread::sleep(boost::posix_time::microseconds(100));

	}



	void Visualization_OccupancyGridCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viz, pcl::visualization::PointCloudColorHandler<PointCustom> &handler, PointCloud<PointCustom> &P, string PCL_name, OccupancyGrid *g) {

		viz->spinOnce();
		viz->removePointCloud(PCL_name);

		g->buildOccupancyGrid(P);
		g->plotOccupancyGridMap();

		CloudConstPtr cloud;
		cloud = P.makeShared();
		handler.setInputCloud(cloud);
		if (!viz->updatePointCloud(cloud, handler, PCL_name)) {
			viz->addPointCloud(cloud, handler, PCL_name);
		}

		P.clear();

		g->resetAll();

		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
	
	
	void Visualization_ElevationGridCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viz, pcl::visualization::PointCloudColorHandler<PointCustom> &handler, PointCloud<PointCustom> &P, string PCL_name, OccupancyGrid *g) {

		viz->spinOnce();
		viz->removePointCloud(PCL_name);

		g->buildOccupancyGridWithMinMaxValues(P);
		g->plotHeightJumpGridMap();

		CloudConstPtr cloud;
		cloud = P.makeShared();
		handler.setInputCloud(cloud);
		if (!viz->updatePointCloud(cloud, handler, PCL_name)) {
			viz->addPointCloud(cloud, handler, PCL_name);
		}

		P.clear();

		g->resetAll();

		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}



};
