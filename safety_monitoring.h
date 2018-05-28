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
	std::vector<int> point_id;   //��դ���е�����
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
	float nx, ny;  //������ƽ�з��������
	float parallel_distance;
	float vertical_distance;
};

class SafetyMonitoring
{
public:
	SafetyMonitoring();
	~SafetyMonitoring();
	/*���ó�ʼ�����ֱ���;*/
	void SetInitialGridResolution(float initial_resolution){ initial_resolution_ = initial_resolution; }
	/*�������ո����ֱ���;*/
	void SetFinalGridResolution(float final_resolution){ final_resolution_ = final_resolution; }
	/*���ñ仯��ֵ;*/
	void SetHeightDiffThreshold(float height_diff_thres){ height_diff_thres_ = height_diff_thres; }
	/*����������������;*/
	void FindForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, float resolution, CloudPtr ref_foreign_region, CloudPtr com_foreign_region);
	/*����դ��ֱ��ʣ�����������������;*/
	CloudPtr RefineForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, RailwayParameters rp);
	/*����������������ȡ��������,�������������;*/
	std::vector<ForeignObject> GetForeignObject(CloudPtr foreign_region);
	

private:
	/*��ȡ����ⷶΧ�ڵĵ���;*/
	void GetWithinCloud(CloudPtr cloud, CloudPtr within_cloud,RailwayParameters rp);
	/*��ȡ���Ʊ߽�;*/
	void GetBound(CloudPtr cloud,Bound* bound);
	/*��ʼ������������ÿ��դ������;*/
	void InitialGrid(CloudPtr ref_cloud, CloudPtr com_cloud, Voxel* ref_grid, Voxel* com_grid, float resolution, float min_x, float min_y, float max_x, float max_y, int list);

	float initial_resolution_;  //��ʼ�����ֱ���
	float final_resolution_;  //���ո����ֱ���
	float height_diff_thres_;  //�仯��ֵ
};
#endif//SAFETY_MONITORING