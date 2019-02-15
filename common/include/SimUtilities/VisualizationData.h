/*! @file GraphicsDebugData.h
 *  @brief Data for sim to graph
 */

#ifndef VISUALIZATION_DATA_H
#define VISUALIZATION_DATA_H

#define VISUALIZATION_MAX_PATH_POINTS 2000
#define VISUALIZATION_MAX_PATHS 10
#define VISUALIZATION_MAX_ITEMS 100
#define VISUALIZATION_MAX_CHEETAHS 0


struct SphereVisualization
{
	double position[3];
	double color[4];
	double radius;
};

struct BlockVisualization
{
	double dimension[3];
	double corner_position[3];
	double rpy[3];
	double color[4];
};

struct ArrowVisualization
{
	double base_position[3];
	double direction[3];
	double color[4];
	double head_width;
	double head_length;
	double shaft_width;
};

struct CheetahVisualization
{
	// TODO

};

struct PathVisualization
{
	size_t num_points;
	double color[4];
	double position[VISUALIZATION_MAX_PATH_POINTS][3];
};

struct ConeVisualization
{
	double point_position[3];
	double direction[3];
	double color[4];
	double radius;
};


struct VisualizationData
{
	size_t num_paths, num_arrows, num_cones, num_spheres, num_blocks;
	SphereVisualization spheres[VISUALIZATION_MAX_ITEMS];
	BlockVisualization blocks[VISUALIZATION_MAX_ITEMS];
	ArrowVisualization arrows[VISUALIZATION_MAX_ITEMS];
	ConeVisualization cones[VISUALIZATION_MAX_ITEMS];
	PathVisualization paths[VISUALIZATION_MAX_PATHS];
};

#endif