#include "OccupancyGrid.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
/*****
Build occupancy grid. = fill occupied cells X & Y coordinates vectors concurently & then store the indexes of points for each occupied cell
****/
void OccupancyGrid::buildOccupancyGrid(pcl::PointCloud<PointCustom> pts) {

	/*find occupied cells indexes*/
	cout << "    " << pts.size() << " AICII" << endl;
	for (int i = 0; i < pts.size(); i++) {
		if (!OccupancyGrid::roi.isInside2D(pts.at(i))) {} //prea departate, nu ma intereseaza
		else {
			int x = OccupancyGrid::obtainCorrespondingRow(pts.at(i).x);
			int y = OccupancyGrid::obtainCorrespondingColumn(pts.at(i).y);
			cout << x << " " << y << endl;
			//check is pair of indexes was not defined yet
			bool is = isCellOccupied(x,y);

			//if not, define it
			if (!is) {
				occupiedCellsXCoords.push_back(x);
				occupiedCellsYCoords.push_back(y);
				occupancyGrid[x][y] = occupiedCellsXCoords.size()-1; 
			}

		}
	}

	totalNoOfOccupiedCells = occupiedCellsXCoords.size();
	cellsPointsIndexes = new vector<int>[totalNoOfOccupiedCells];

	/*store indexes of points for each occupied cell*/
	for (int i = 0; i < pts.size(); i++) {
		if (!OccupancyGrid::roi.isInside2D(pts.at(i))) {} //prea departate, nu ma intereseaza
		else {
			int x = OccupancyGrid::obtainCorrespondingRow(pts.at(i).x);
			int y = OccupancyGrid::obtainCorrespondingColumn(pts.at(i).y);

			//check is pair of indexes was not defined yet
			int pos = 0;
			bool is = isCellOccupied(x,y);

			//if found pos
			if (is) {
				cellsPointsIndexes[getCellIndex(x,y)].push_back(i);
			}

		}
	}
}
//and store min max values per cell
void OccupancyGrid::buildOccupancyGridWithMinMaxValues(pcl::PointCloud<PointCustom> pts) {

	/*find occupied cells indexes*/
	for (int i = 0; i < pts.size(); i++) {
		if (!OccupancyGrid::roi.isInside2D(pts.at(i))) {} //prea departate, nu ma intereseaza
		else {
			int x = OccupancyGrid::obtainCorrespondingRow(pts.at(i).x);
			int y = OccupancyGrid::obtainCorrespondingColumn(pts.at(i).y);

			//check is pair of indexes was not defined yet
			bool is = isCellOccupied(x, y);

			//if not, define it
			if (!is) {
				occupiedCellsXCoords.push_back(x);
				occupiedCellsYCoords.push_back(y);
				occupancyGrid[x][y] = occupiedCellsXCoords.size() - 1;
			}

			if (pts.at(i).z > topHeights[x][y]) {
				topHeights[x][y] = pts.at(i).z;
			}
			if (pts.at(i).z < baseHeights[x][y]) {
				baseHeights[x][y] = pts.at(i).z;
			}

		}
	}

	totalNoOfOccupiedCells = occupiedCellsXCoords.size();
	cellsPointsIndexes = new vector<int>[totalNoOfOccupiedCells];

	/*store indexes of points for each occupied cell*/
	for (int i = 0; i < pts.size(); i++) {
		if (!OccupancyGrid::roi.isInside2D(pts.at(i))) {} //prea departate, nu ma intereseaza
		else {
			int x = OccupancyGrid::obtainCorrespondingRow(pts.at(i).x);
			int y = OccupancyGrid::obtainCorrespondingColumn(pts.at(i).y);

			//check is pair of indexes was not defined yet
			int pos = 0;
			bool is = isCellOccupied(x, y);

			//if found pos
			if (is) {
				cellsPointsIndexes[getCellIndex(x, y)].push_back(i);
			}

		}
	}
}

bool  OccupancyGrid::isCellOccupied(int x, int y) {
	return occupancyGrid[x][y] > -1;
}

int  OccupancyGrid::getCellIndex(int x, int y) {
	return occupancyGrid[x][y];
}


void OccupancyGrid::resetAll() {
	for (int i = 0; i < totalNoOfOccupiedCells; i++) {
		cellsPointsIndexes[i].clear();
	}
	occupiedCellsXCoords.clear();
	occupiedCellsYCoords.clear();

	for (int i = 0; i< OccupancyGrid::gridWidth; i++)
		for (int j = 0; j < OccupancyGrid::gridHeight; j++) {
			occupancyGrid[i][j] = -1;
		}
}

void OccupancyGrid::plotOccupancyGridMap() {
	cv::Mat m(gridHeight, gridWidth, CV_8UC3, cv::Scalar(255, 255, 255));

	for (int i = 0; i< gridWidth; i++)
		for (int j = 0; j < gridHeight; j++) {
			if (isCellOccupied(i, j) && occupancyGrid[i][j] >40) {
				m.at<cv::Vec3b>(i, j) = 0;
			}
		}

	cv::imshow("OccupancyMap", m);

}


cv::Mat OccupancyGrid::getImageofOccupancyGridMap() {
	cv::Mat m(gridHeight, gridWidth, CV_8UC3, cv::Scalar(255, 255, 255));

	for (int i = 0; i< gridWidth; i++)
		for (int j = 0; j < gridHeight; j++) {
			if (isCellOccupied(i, j)) {
				m.at<cv::Vec3b>(i, j) = 0;
			}
		}

	return m;

}

void OccupancyGrid::plotHeightJumpGridMap(double maxTopHeight, double maxBaseHeight) {
	cv::Mat m(gridHeight, gridWidth, CV_8UC3, cv::Scalar(255, 255, 255));

	for (int i = 0; i< gridWidth; i++)
		for (int j = 0; j < gridHeight; j++) {
			if (isCellOccupied(i, j)) {
				if(topHeights[i][j] <= maxTopHeight && baseHeights[i][j]>=maxBaseHeight)
					m.at<cv::Vec3b>(i, j) = 0;
			}
		}

	cv::imshow("HeightJumpMap", m);

}

cv::Mat OccupancyGrid::getHeightJumpGridMap(double maxTopHeight, double maxBaseHeight,double filterL) {
	cv::Mat m(gridHeight, gridWidth, CV_8UC3, cv::Scalar(255, 255, 255));

	for (int i = 0; i< gridWidth; i++)
		for (int j = 0; j < gridHeight; j++) {
			if (isCellOccupied(i, j)) {
				double heightJump = topHeights[i][j] - baseHeights[i][j];
				if (heightJump <= maxTopHeight && heightJump >= maxBaseHeight) {
					double fi = (heightJump - maxBaseHeight) / (maxTopHeight - maxBaseHeight)*255;
					if (fi >= filterL) {
						m.at<cv::Vec3b>(i, j)[0] = fi;
						m.at<cv::Vec3b>(i, j)[1] = fi;
						m.at<cv::Vec3b>(i, j)[2] = fi;
					}
				}
			}
		}

	return m;

}



int OccupancyGrid::getHeight() const
{
	return gridHeight;
}

int OccupancyGrid::getWidth() const
{
	return gridWidth;
}

bool OccupancyGrid::isInMap(const PointCustom point) {
	return BETWEEN(point.y, roi.minY, roi.maxY)
		&& BETWEEN(point.x, roi.minX, roi.maxX);
}

double OccupancyGrid::calculateGridCenterX(int j) const {
	return roi.minX + j * resolution + resolution / 2;
}

double OccupancyGrid::calculateGridCenterY(int i) const {
	return roi.minY + i * resolution + resolution / 2;
}

int OccupancyGrid::obtainCorrespondingColumn(double y) const {
	return (int)((y - roi.minY) / resolution);

}

int OccupancyGrid::obtainCorrespondingRow(double x) const {
	return (int)((x - roi.minX) / resolution);
}

double OccupancyGrid::calculateCellArea() {
	return resolution * resolution;
}

bool OccupancyGrid::isInRange(int row, int col) const
{
	return (row >= 0 && row < gridWidth && col >= 0 && col < gridHeight);
}


