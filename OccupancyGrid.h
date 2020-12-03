#pragma once
#include "ROI.h"
#include "GridMap.h"
#include "custom_point.hpp"
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;


class OccupancyGrid
{
protected:
	int totalNoOfOccupiedCells;

	
	

	double **baseHeights;
	double **topHeights;

	//it will store -1 if not occupied and >-1 if occupied. value = the index of occupiedCellXCoords
	
  
	double resolution; //cell dimension
	ROI roi;	   /* Represents the area represented by the map. */


public:
	int gridWidth;
	int gridHeight;
	int **occupancyGrid;
	vector<int> *cellsPointsIndexes;
	vector<int> occupiedCellsXCoords;
	vector<int> occupiedCellsYCoords;

	void initializeAll() { 
		occupiedCellsXCoords = vector<int>();
		occupiedCellsYCoords = vector<int>();

		occupancyGrid = new int*[gridHeight];

		for (int i = 0; i<gridHeight; i++) {
			occupancyGrid[i] = new int[gridWidth];
		}

		baseHeights = new double*[gridHeight];

		for (int i = 0; i<gridHeight; i++) {
			baseHeights[i] = new double[gridWidth];
		}

		topHeights = new double*[gridHeight];

		for (int i = 0; i<gridHeight; i++) {
			topHeights[i] = new double[gridWidth];
		}

		for (int i = 0; i< gridWidth; i++)
			for (int j = 0; j < gridHeight; j++) {
				occupancyGrid[i][j] = -1;
				baseHeights[i][j] = 9999;
				topHeights[i][j] = -9999;
			}
	}

	OccupancyGrid( double resolution, double minX, double minY, double maxX, double maxY) {
		roi.maxX = maxX;
		roi.maxY = maxY;
		roi.minX = minX;
		roi.minY = minY;

		int gridWidth = (double)abs(maxY -minY) / resolution; 
		int gridHeight = (double)abs(maxX - minX) / resolution;

		this->gridWidth = gridWidth;
		this->gridHeight = gridHeight;

		this->resolution = resolution;
		totalNoOfOccupiedCells = 0;
		initializeAll();
	}

	~OccupancyGrid(void) {
		OccupancyGrid::resetAll();

		for (int i = 0; i<gridHeight; i++) {
			delete[] occupancyGrid[i];
			delete[] baseHeights[i];
			delete[] topHeights[i];
		}

		delete[] occupancyGrid;
		delete[] baseHeights;
		delete[] topHeights;

		delete[] cellsPointsIndexes;
	}



	void buildOccupancyGrid(pcl::PointCloud<PointCustom> pts);
	

	void buildOccupancyGridWithMinMaxValues(pcl::PointCloud<PointCustom> pts);

	bool isCellOccupied(int x, int y);

	int  getCellIndex(int x, int y);

	void resetAll();

	cv::Mat getImageofOccupancyGridMap();

	void plotOccupancyGridMap();
	void plotHeightJumpGridMap(double maxTopHeight, double minTopHeight);

	cv::Mat getHeightJumpGridMap(double maxTopHeight, double maxBaseHeight,  double filterL);

	int getHeight() const;
	int getWidth() const;

	bool isInRange(int row, int col) const;

	bool isInMap(const PointCustom point);

	double calculateGridCenterX(int j) const;
	double calculateGridCenterY(int i) const;

	int obtainCorrespondingColumn(double y) const;
	int obtainCorrespondingRow(double x) const;

	/* Calculates the area corresponding to a cell in mm^2.*/
	double calculateCellArea();
};