#pragma once
#include "pcl/pcl_config.h"
#include "custom_point.hpp"
#include <fstream>
using namespace std;

#define NR_0s_FILE_TITLE 10
#define FOLDER "G:/Selma/dataset/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/"

class BinFileReader{
public:

	void getCloud(int number, pcl::PointCloud<PointCustom> &my) {
		int aux_n = number; //get nr cifre
		int nrDigits = 0;
		while (aux_n != 0) {
			nrDigits++;
			aux_n /= 10;
		}
		int nr0s = NR_0s_FILE_TITLE - nrDigits;
		std::string _0s = "0";
		std::string filename = FOLDER;
		for (int i = 1; i < nr0s; i++) {
			_0s += "0";
		}
		filename += _0s;
		if (number != 0)
			filename += std::to_string(number);
		filename += ".bin";


		float *velodyneData = new float[1000000];
		FILE *stream = fopen(filename.c_str(), "rb");
		int num = fread(velodyneData, sizeof(float), 1000000, stream) / 4;
		fclose(stream);

		my.clear();
		for (int i = 0; i < num; i++) {

			PointCustom auxiliar = PointCustom();

			auxiliar.intensity = velodyneData[i * 4 + 3];
			auxiliar.x = velodyneData[i*4+0];
			auxiliar.y = velodyneData[i*4 + 1];
			auxiliar.z = velodyneData[i*4 + 2];

			my.push_back(auxiliar);
		}
		delete[] velodyneData;
	}

};