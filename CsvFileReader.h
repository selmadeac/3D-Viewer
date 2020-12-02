#pragma once

#include "pcl/pcl_config.h"
#include "custom_point.hpp"
#include<fstream>
#include <dirent.h>
#include <string>
#include <stdlib.h>

using namespace std;
/*
* Reads a CSV file - applyies transforms on points(rotations or translations) - if any - and returns a Point Cloud array
*/
class ReadCSVFiles {
	
private:
	Eigen::Matrix4d T_mat;
	typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
	
	
public:

/* Reads a CSV file with 3D points organised as follows: the rows represent the 3D points, and the columns represent information about each point
*  IN: filename - path to the csv file
* IN: apply_transforms - true if the transform matrix T_mat is set, false if not(default value)
* OUT: my_cloud - the created cloud array
*/

void getCloudVeloFromCSV(string filename, bool apply_transforms = false, pcl::PointCloud<PointCustom> &my_cloud) {
	std::ifstream file(filename);

	string line = "";
	int n_Lines = 0;

	PointCustom auxiliar = PointCustom();
	auxiliar.reset();
	bool first = true;
	while (true) {
		if (file.eof() || !file.is_open()) break;
		line = "";
		getline(file, line);
		boost::char_separator<char> sep{ "," };
		tokenizer tok{ line, sep };

		if (first == false) { //ignore first line of file because it contains the name of the  columns
			int i = 0;

			for (const auto &t : tok) {
				string aux = t;
				switch (i) {
				case 0:		auxiliar.row = atoi(aux.c_str()); break; //row
				case 1:		auxiliar.coll = atoi(aux.c_str()); break; //coll
				case 2:		break; //targetid?
				case 3:		auxiliar.x = stod(aux.c_str()); break; //x
				case 4:		auxiliar.y = stod(aux.c_str()); break; //y
				case 5:		auxiliar.z = stod(aux.c_str()); break; //z
				case 6:		auxiliar.intensity = stod(aux.c_str()); break; // //intensity
				case 7:		auxiliar.timestamp = stoll(aux.c_str()); break; //timestamp				
				}
				i++;
			}
			PointCustom s = rot_transf_point_velodyne(type, auxiliar.x, auxiliar.y, auxiliar.z);
			auxiliar.x = s.x;
			auxiliar.y = s.y;
			auxiliar.z = s.z;


			double temporar = 0;
			temporar = atan2(s.y, s.x);
			if (temporar < 0)
				temporar += 2 * M_PI;
			auxiliar.yaw = (temporar * 180.0) / M_PI;

			my_cloud.push_back(auxiliar);

		}
		first = false;
	}

	file.close();
}

	/*
	* Apply transforms to point - if any
	*/
PointCustom rot_transf_point_velodyne( float x, float y, float z) {
	Eigen::Vector4d aux(x, y, z, 1);
	Eigen::Vector4d rez;

	rez = T_mat*aux;	

	PointCustom fin = PointCustom();

	fin.x = rez(0);
	fin.y = rez(1);
	fin.z = rez(2);

	return fin;
}

};
