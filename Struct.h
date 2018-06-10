#ifndef STRUCT_H
#define STRUCT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

struct Voxel
{
	std::vector<int> point_id;   //该栅格中点索引
	float max_z;

	Voxel()
	{
		max_z = -FLT_MAX;
	}
};

struct Bound
{
	float min_x;
	float min_y;
	float max_x;
	float max_y;
	float min_z;
	float max_z;
	Bound()
	{
		min_x = min_y = max_x = max_y = min_z = max_z = 0.f;
	}
};

struct ForeignObject
{
	CloudPtr cloud;
	Bound bound;
	float center_x;
	float center_y;
	float area;  //物体占地面积
	float height;  //物体超出轨面高度
	bool between_railway; //是否位于铁轨上
	int position; //一对铁轨由近及远分别为1,2,3,4,5,6（2,5分别位于铁轨上）
	float distance; //相对最近邻铁轨的垂直距离（位于铁轨上时为0）
	
	ForeignObject() :cloud(new Cloud)
	{
		position = -1;
		height = -FLT_MAX;
		distance = 0.f;
	}
};

struct Region
{
	int count;  //铁轨条数
	std::vector<Point> vertexes;
	std::vector<std::vector<Point>> corners;
	float distance;
	Region()
	{
		count = -1;
		distance = 0.3f;
	}
};

struct LinePara
{
	float k;
	float b_max;
	float b_min;
	LinePara()
	{
		k = b_max = b_min = 0.f;
	}
};

struct PlanePara
{
	float a;
	float b;
	float c;
	float d;
	PlanePara()
	{
		a = b = c = d = 0.f;
	}
};

struct RailwayRect
{
	LinePara parallel_line_para;
	LinePara vertical_line_para;
	PlanePara plane_para;
	float extend;
	float interval;
};

#endif //STRUCT_H