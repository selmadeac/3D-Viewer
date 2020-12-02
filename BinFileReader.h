#pragma once
#include "pcl/pcl_config.h"
#include "custom_point.hpp"
#include <fstream>
using namespace std;

#define NR_0s_FILE_TITLE 10

class BinFileReader{
public:

	void getCloud(std::string filepath, pcl::PointCloud<PointCustom> &my_cloud) {
		std::string filename = filepath


		float *velodyneData = new float[1000000];
		FILE *stream = fopen(filename.c_str(), "rb");
		int num = fread(velodyneData, sizeof(float), 1000000, stream) / 4;
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
