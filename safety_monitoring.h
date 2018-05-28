#ifndef SAFETY_MONITORING_H
#define SAFETY_MONITORING_H

#include "stdafx.h"
#include <vector>
#include <math.h>
#include <set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

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
	float height;
	ForeignObject() :cloud(new Cloud)
	{
		height = 0.f;
	}
};

struct RailwayParameters
{
	std::vector<float> parallel_x;  
	std::vector<float> parallel_y;
	float vertical_x1, vertical_y1, vertical_x2, vertical_y2;
	float nx, ny;  //与铁轨平行方向的向量
	float parallel_distance;
	float vertical_distance;
};

class SafetyMonitoring
{
public:
	SafetyMonitoring();
	~SafetyMonitoring();
	/*设置初始格网分辨率;*/
	void SetInitialGridResolution(float initial_resolution){ initial_resolution_ = initial_resolution; }
	/*设置最终格网分辨率;*/
	void SetFinalGridResolution(float final_resolution){ final_resolution_ = final_resolution; }
	/*设置变化阈值;*/
	void SetHeightDiffThreshold(float height_diff_thres){ height_diff_thres_ = height_diff_thres; }
	/*查找疑似异物区域;*/
	void FindForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, float resolution, CloudPtr ref_foreign_region, CloudPtr com_foreign_region);
	/*降低栅格分辨率，精化疑似异物区域;*/
	CloudPtr RefineForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, RailwayParameters rp);
	/*从疑似异物区域提取单个异物,并返回异物个数;*/
	std::vector<ForeignObject> GetForeignObject(CloudPtr foreign_region);
	

private:
	/*截取待检测范围内的点云;*/
	void GetWithinCloud(CloudPtr cloud, CloudPtr within_cloud,RailwayParameters rp);
	/*获取点云边界;*/
	void GetBound(CloudPtr cloud,Bound* bound);
	/*初始化格网，计算每个栅格属性;*/
	void InitialGrid(CloudPtr ref_cloud, CloudPtr com_cloud, Voxel* ref_grid, Voxel* com_grid, float resolution, float min_x, float min_y, float max_x, float max_y, int list);

	float initial_resolution_;  //初始格网分辨率
	float final_resolution_;  //最终格网分辨率
	float height_diff_thres_;  //变化阈值
};
#endif//SAFETY_MONITORING