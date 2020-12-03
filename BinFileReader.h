#pragma once
#include "pcl/pcl_config.h"
#include "custom_point.hpp"
#include <fstream>
using namespace std;


class BinFileReader{
public:

	void getCloudVeloFromBin(std::string filepath, pcl::PointCloud<PointCustom> &my_cloud) {
		std::string filename = filepath

		float *velodyneData = new float[MAX_NUMBER_OF_POINTS];
		FILE *stream = fopen(filename.c_str(), "rb");
		int num = fread(velodyneData, sizeof(float), MAX_NUMBER_OF_POINTS, stream) / 4;
		fclose(stream);

		my.clear();
		for (int i = 0; i < num; i++) {
			PointCustom auxiliar = PointCustom();

			auxiliar.intensity = velodyneData[i * 4 + 3];
			auxiliar.x = velodyneData[i*4 + 0];
			auxiliar.y = velodyneData[i*4 + 1];
			auxiliar.z = velodyneData[i*4 + 2];

			my_cloud.push_back(auxiliar);
		}
		delete[] velodyneData;
	}

};