# 3D-Viewer
3D viewer is a free framework for visualizing a 3D point clouds coming from a Velodyne sensor writtten in C++.

It meets the following functional requirements:
- Given a binary file or a csv file with 3D points and auxiliary points information, the system reads the file and stores the 3D points in an array.
- The system represents each point as a structure with main (x,y,z,intensity) and auxiliary attributes(index)
- The system can represent the space as either point cloud, grid maps (occupancy grid, elevation map) or voxels (cuboid or multi volumetric grid)

TO DO:
-voxel representation

The used Coordinate system:
Standard for PCL Library

Dependencies:
- PointCloud Library
- Eigen 3.0
- OpenCV
