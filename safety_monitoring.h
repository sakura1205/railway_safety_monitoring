#ifndef SAFETY_MONITORING_H
#define SAFETY_MONITORING_H

#include "stdafx.h"
#include <vector>
#include <math.h>
#include <set>
#include "Struct.h"

#include <pcl/io/pcd_io.h>

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
	CloudPtr RefineForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, Region region);
	/*从疑似异物区域提取单个异物,并返回异物个数;*/
	std::vector<ForeignObject> GetForeignObject(CloudPtr foreign_region,CloudPtr railway_cloud);
	

private:
	/*截取待检测范围内的点云(以扫描仪为原点，左下角为p1，右下角为p2,左上角为p3,右上角为p4，即p1p2,p3p4为铁轨方向);*/
	void GetWithinCloud(CloudPtr cloud,CloudPtr within_cloud, std::vector<Point> vertexes, float distance);
	/*截取指定范围（RailwayRect）内的点;*/
	void GetWithinCloud(CloudPtr cloud, CloudPtr within_cloud, RailwayRect railway_rect);
	/*判断异物是否位于铁轨上;*/
	bool IsOnRailway(CloudPtr foreign_cloud, CloudPtr railway_cloud);
	/*判断异物位置及相对于最近邻铁轨的垂直距离，interval为一对铁轨之间的间距;*/
	void GetObjectPosition(ForeignObject& foreign_object, std::vector<std::vector<RailwayRect>> railway_rects,CloudPtr railway_cloud);
	/*计算异物面积;*/
	float GetObjectArea(CloudPtr cloud,float resolution = 0.01);
	/*计算异物所在位置铁轨切平面Z值;*/
	float GetRailwayPlaneHeight(ForeignObject foreign_object, std::vector<std::vector<RailwayRect>> railway_rects);
	/*计算点是否超出铁轨平面;*/
	float ComputeVerticalDistanceToPlane(Point pt, std::vector<std::vector<RailwayRect>> railway_rects);
	/*判断点是否位于给定rectrangle内;*/
	bool IsInsideRect(float x, float y, RailwayRect railway_rect);
	/*判断点是否位于拓宽后的rectangle内;*/
	bool IsInsideExtendedRect(float x, float y, RailwayRect railway_rect);
	/*计算点到直线距离;*/
	float ComputeDistanceFromPointToLine(float x, float y, float k,float b);
	/*计算点云到直线的最短距离;*/
	float ComputeDistanceFromCloudToLine(CloudPtr cloud, float k, float b);
	/*获取点云边界;*/
	void GetBound(CloudPtr cloud,Bound* bound);
	/*初始化格网，计算每个栅格属性;*/
	void InitialGrid(CloudPtr ref_cloud, CloudPtr com_cloud, Voxel* ref_grid, Voxel* com_grid, float resolution, float min_x, float min_y, float max_x, float max_y, int list);
	/*计算铁轨分段后每个rectangle的参数（直线和平面拟合参数）*/
	void ComputeRailwayRectangle(Region region, std::vector<std::vector<RailwayRect>> railway_rects);

	float initial_resolution_;  //初始格网分辨率
	float final_resolution_;  //最终格网分辨率
	float height_diff_thres_;  //变化阈值
	std::vector<std::vector<RailwayRect>> railway_rects_; //铁轨分段矩形
};
#endif//SAFETY_MONITORING