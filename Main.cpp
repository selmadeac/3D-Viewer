#define PCL_NO_PRECOMPILE

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/console/parse.h>
#include "custom_point.hpp"
#include "SystemViewer.h"


using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;



int main(int argc, char ** argv)
{
	pcl::visualization::PointCloudColorHandlerGenericField<PointCustom> color_handler("label");

	SystemViewer view(color_handler);
	view.run();

	return (0);
}
