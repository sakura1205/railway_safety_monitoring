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

	/*���ó�ʼ�����ֱ���;*/
	void SetInitialGridResolution(float initial_resolution){ initial_resolution_ = initial_resolution; }
	/*�������ո����ֱ���;*/
	void SetFinalGridResolution(float final_resolution){ final_resolution_ = final_resolution; }
	/*���ñ仯��ֵ;*/
	void SetHeightDiffThreshold(float height_diff_thres){ height_diff_thres_ = height_diff_thres; }
	/*����������������;*/
	void FindForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, float resolution, CloudPtr ref_foreign_region, CloudPtr com_foreign_region);
	/*����դ��ֱ��ʣ�����������������;*/
	CloudPtr RefineForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, Region region);
	/*����������������ȡ��������,�������������;*/
	std::vector<ForeignObject> GetForeignObject(CloudPtr foreign_region,CloudPtr railway_cloud);
	

private:
	/*��ȡ����ⷶΧ�ڵĵ���(��ɨ����Ϊԭ�㣬���½�Ϊp1�����½�Ϊp2,���Ͻ�Ϊp3,���Ͻ�Ϊp4����p1p2,p3p4Ϊ���췽��);*/
	void GetWithinCloud(CloudPtr cloud,CloudPtr within_cloud, std::vector<Point> vertexes, float distance);
	/*��ȡָ����Χ��RailwayRect���ڵĵ�;*/
	void GetWithinCloud(CloudPtr cloud, CloudPtr within_cloud, RailwayRect railway_rect);
	/*�ж������Ƿ�λ��������;*/
	bool IsOnRailway(CloudPtr foreign_cloud, CloudPtr railway_cloud);
	/*�ж�����λ�ü���������������Ĵ�ֱ���룬intervalΪһ������֮��ļ��;*/
	void GetObjectPosition(ForeignObject& foreign_object, std::vector<std::vector<RailwayRect>> railway_rects,CloudPtr railway_cloud);
	/*�����������;*/
	float GetObjectArea(CloudPtr cloud,float resolution = 0.01);
	/*������������λ��������ƽ��Zֵ;*/
	float GetRailwayPlaneHeight(ForeignObject foreign_object, std::vector<std::vector<RailwayRect>> railway_rects);
	/*������Ƿ񳬳�����ƽ��;*/
	float ComputeVerticalDistanceToPlane(Point pt, std::vector<std::vector<RailwayRect>> railway_rects);
	/*�жϵ��Ƿ�λ�ڸ���rectrangle��;*/
	bool IsInsideRect(float x, float y, RailwayRect railway_rect);
	/*�жϵ��Ƿ�λ���ؿ���rectangle��;*/
	bool IsInsideExtendedRect(float x, float y, RailwayRect railway_rect);
	/*����㵽ֱ�߾���;*/
	float ComputeDistanceFromPointToLine(float x, float y, float k,float b);
	/*������Ƶ�ֱ�ߵ���̾���;*/
	float ComputeDistanceFromCloudToLine(CloudPtr cloud, float k, float b);
	/*��ȡ���Ʊ߽�;*/
	void GetBound(CloudPtr cloud,Bound* bound);
	/*��ʼ������������ÿ��դ������;*/
	void InitialGrid(CloudPtr ref_cloud, CloudPtr com_cloud, Voxel* ref_grid, Voxel* com_grid, float resolution, float min_x, float min_y, float max_x, float max_y, int list);
	/*��������ֶκ�ÿ��rectangle�Ĳ�����ֱ�ߺ�ƽ����ϲ�����*/
	void ComputeRailwayRectangle(Region region, std::vector<std::vector<RailwayRect>> railway_rects);

	float initial_resolution_;  //��ʼ�����ֱ���
	float final_resolution_;  //���ո����ֱ���
	float height_diff_thres_;  //�仯��ֵ
	std::vector<std::vector<RailwayRect>> railway_rects_; //����ֶξ���
};
#endif//SAFETY_MONITORING