/*
Point2d Cloud Library Custom Extension
Florea Horatiu Razvan
UTCN June 2016

Point2d type definition containing timestamp
*/
#include "pcl/pcl_config.h"

#ifndef CUSTOM_POINT_H_
#define CUSTOM_POINT_H_

#define PCL_NO_PRECOMPILE

#include <pcl/io/hdl_grabber.h>
#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PointCustom {
	float x;
	float y;
	float z;
	double intensity;
	long long timestamp;
	uint16_t label;
	int labelGT; //ground truth label
	float yaw;
	float pitch;
	int row;
	int coll;
	int8_t cam_labels[4];//"CAMF", "CAMR", "CAMB", "CAML"
	int lidarType;


	PointCustom(float xx = 0, float yy = 0, float zz = 0, double intens = 0, uint16_t lab = 0) : x(xx), y(yy), z(zz), intensity(intens), label(lab) {	};

	PointCustom& operator=(const PointCustom& b)
	{
		x = b.x;
		y = b.y;
		z = b.z;
		timestamp = b.timestamp;
		intensity = b.intensity;
		label = b.label;
		labelGT = b.labelGT;
		row = b.row;
		coll = b.coll;
		lidarType = b.lidarType;
		cam_labels[0] = b.cam_labels[0];
		cam_labels[1] = b.cam_labels[1];
		cam_labels[2] = b.cam_labels[2];
		cam_labels[3] = b.cam_labels[3];
		lidarType = b.lidarType;
		return *this;
	}

	void reset()
	{
		this->x = 0.0;
		this->y = 0.0;
		this->z = 0.0;
		this->intensity = 0;
		this->timestamp = 0.0;

		cam_labels[0] = -1;
		cam_labels[1] = -1;
		cam_labels[2] = -1;
		cam_labels[3] = -1;

		labelGT = -1;
		row = -1;
		coll = -1;
		label = 0;
	}


	PointCustom operator+(const PointCustom& b)
	{
		x += b.x;
		y += b.y;
		z += b.z;
		this->intensity = b.intensity;
		this->label = b.label;
		lidarType = b.lidarType;
		row = b.row;
		coll = b.coll;
		return *this;
	}

	bool operator==(const PointCustom& b)
	{
		bool aux = false;
		if (x == b.x && y == b.y && z == b.z &&
			timestamp == b.timestamp &&
			intensity == b.intensity &&label == b.label)
			return true;
		else
			return false;
	}

	PointCustom CrossProduct(PointCustom& b)
	{
		PointCustom cross = PointCustom();
		cross.x = y*b.z - z * b.y;
		cross.y = z*b.x - x * b.z;
		cross.z = x*b.y - y*b.x;
		return cross;
	}

	PointCustom operator-(const PointCustom& b)
	{
		x -= b.x;
		y -= b.y;
		z -= b.z;
		this->intensity = b.intensity;
		this->label = b.label;
		lidarType = b.lidarType;
		row = b.row;
		coll = b.coll;
		return *this;
	}

	PointCustom operator/(const double& b) //de ce?..ce!?
	{
		x /= (double)b;
		y /= (double)b;
		z /= (double)b;

		return *this;
	}
	PointCustom operator*(const int& b)
	{
		x *= b;
		y *= b;
		z *= b;
		return *this;
	}
	friend std::ostream& operator<<(std::ostream& os, const PointCustom& dt)
	{
		os << dt.x<< ' / ' << dt.y << ' / ' << dt.z;
		return os;
	}

	bool valid() {
		if (x != 0 && y != 0 && z != 0)
			return true;
		return false;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointCustom,
(float, x, x)
(float, y, y)
(float, z, z)
(double, intensity, intensity)
(double, timestamp, timestamp)
(uint16_t, label, label))

typedef struct pair_int {
	int low;
	int high;
	pair_int(int a, int b) {
		low = a;
		high = b;
	}
	pair_int() {

	}
};

#endif /* TIMESTAMP_POINT_H_ */
