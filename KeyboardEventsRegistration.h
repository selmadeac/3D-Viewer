#pragma once
#include <iostream>
#include<string>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>

using namespace pcl;
using namespace std;
  

int visualization_type = 0;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	string str = event.getKeySym();
	bool keyDown = 1;

	if (str == (string("g")) && keyDown) {
		std::cout << "'g' was pressed" << std::endl;
		visualization_type = 1;
	}

	if (str == (string("o")) && keyDown) {
		std::cout << "'o' was pressed" << std::endl;
		visualization_type = 0;
	}
	if (str == (string("s")) && keyDown) {
		std::cout << "stop cloud was pressed" << std::endl;
		visualization_type = 2;
	}

	if (str == (string("q")) && keyDown) {
		std::cout << "stop cloud was pressed" << std::endl;
		visualization_type = 3;
	}

}
