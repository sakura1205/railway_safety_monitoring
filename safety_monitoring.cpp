#include "stdafx.h"
#include "safety_monitoring.h"

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
SafetyMonitoring::SafetyMonitoring()
{
	initial_resolution_ = 0.2;
	final_resolution_ = 0.02;
	height_diff_thres_ = 0.07;
}

SafetyMonitoring::~SafetyMonitoring()
{

}

void SafetyMonitoring::FindForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud, float resolution,CloudPtr ref_foreign_region,CloudPtr com_foreign_region)
{
	//计算格网边界
	Bound* bound = new Bound();
	GetBound(com_cloud, bound);

	//计算格网行列数及栅格总数
	int row = ceil((bound->max_y - bound->min_y) / resolution);
	int list = ceil((bound->max_x - bound->min_x) / resolution);
	int voxel_count = row * list;

	//初始化格网
	Voxel* ref_grid = new Voxel[voxel_count];
	Voxel* com_grid = new Voxel[voxel_count];

	InitialGrid(ref_cloud, com_cloud, ref_grid, com_grid, resolution,bound->min_x,bound->min_y,bound->max_x,bound->max_y,list);

	std::vector<int> region_voxel_id;

	//查询异物所在栅格
	for (int i = 0; i < voxel_count; ++i)
	{
		if ((com_grid[i].max_z - ref_grid[i].max_z) > height_diff_thres_)
		{
			region_voxel_id.push_back(i);
		}			
	}

	//计算异物点点云
	std::vector<int> ref_foreign_region_point_id;
	std::vector<int> com_foreign_region_point_id;
	for (int i = 0; i < region_voxel_id.size(); ++i)
	{
		ref_foreign_region_point_id.insert(ref_foreign_region_point_id.end(), ref_grid[region_voxel_id[i]].point_id.begin(), ref_grid[region_voxel_id[i]].point_id.end());
		com_foreign_region_point_id.insert(com_foreign_region_point_id.end(), com_grid[region_voxel_id[i]].point_id.begin(), com_grid[region_voxel_id[i]].point_id.end());
	}

	pcl::copyPointCloud(*ref_cloud, ref_foreign_region_point_id, *ref_foreign_region);
	pcl::copyPointCloud(*com_cloud, com_foreign_region_point_id, *com_foreign_region);

	ref_foreign_region_point_id.clear();
	std::vector<int>().swap(ref_foreign_region_point_id);
	com_foreign_region_point_id.clear();
	std::vector<int>().swap(com_foreign_region_point_id);
}

CloudPtr SafetyMonitoring::RefineForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud,Region region)
{
	//截取待检测初始范围内的点云
	CloudPtr ref_within_cloud(new Cloud);
	CloudPtr com_within_cloud(new Cloud);
	GetWithinCloud(ref_cloud, ref_within_cloud,region.vertexes,0);
	GetWithinCloud(com_cloud, com_within_cloud, region.vertexes,0);
 	pcl::io::savePCDFile("E:/com_within_cloud.pcd", *com_within_cloud);
 	pcl::io::savePCDFile("E:/ref_within_cloud.pcd", *ref_within_cloud);

	//寻找异物区域
	for (float resolution = initial_resolution_; resolution > final_resolution_; resolution /= 2)
	{
		CloudPtr ref_foreign_region(new Cloud);
		CloudPtr com_foreign_region(new Cloud);
		FindForeignRegion(ref_within_cloud, com_within_cloud,resolution,ref_foreign_region,com_foreign_region);
		ref_within_cloud = ref_foreign_region;
 		com_within_cloud = com_foreign_region;
	}

	ComputeRailwayRectangle(region, railway_rects_);
	
	//判断异物区域是否在铁轨两侧30cm范围内
	CloudPtr foreign_region(new Cloud);
	for (int i = 0; i < railway_rects_.size(); ++i)
	{
		for (int j = 0; j < railway_rects_[i].size(); ++j)
		{
			CloudPtr temp_cloud(new Cloud);
			GetWithinCloud(com_within_cloud, temp_cloud, railway_rects_[i][j]);
			*foreign_region += *temp_cloud;
		}
	}
// 	for (int i = 0; i < region.corners.size(); i += 2)
// 	{
// 		for (int j = 0; j < region.corners[i].size()-1; ++j)
// 		{
// 			std::vector<Point> rectangle;
// 			Point p1 = region.corners[i][j];
// 			Point p2 = region.corners[i][j + 1];
// 			Point p3 = region.corners[i + 1][j];
// 			Point p4 = region.corners[i + 1][j + 1];
// 			rectangle.push_back(p1);
// 			rectangle.push_back(p2);
// 			rectangle.push_back(p3);
// 			rectangle.push_back(p4);
// 			CloudPtr temp_cloud(new Cloud);
// 			GetWithinCloud(com_within_cloud, temp_cloud, rectangle, region.distance);
// 			*foreign_region += *temp_cloud;
// 		}
// 	}
	pcl::io::savePCDFile("E:/foreign_region.pcd", *com_within_cloud);
	pcl::io::savePCDFile("E:/foreign_region_within.pcd", *foreign_region);
	return foreign_region;
}

std::vector<ForeignObject> SafetyMonitoring::GetForeignObject(CloudPtr foreign_region,CloudPtr railway_cloud)
{
	std::vector<ForeignObject> foreign_objects;

	//获取单个异物点索引
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(foreign_region);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	CloudPtr foreign_objects_cloud(new Cloud);
	ec.setClusterTolerance(0.1); // 10cm
	ec.setMinClusterSize(25);
	ec.setMaxClusterSize(10000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(foreign_region);
	ec.extract(cluster_indices);

	//获取异物信息
	for (int i = 0; i < cluster_indices.size(); ++i)
	{
		ForeignObject foreign_object;
		CloudPtr foreign_object_cloud(new Cloud);
		pcl::copyPointCloud(*foreign_region, cluster_indices[i], *foreign_object_cloud);
		pcl::io::savePCDFile("E:/foreign_object_cloud.pcd", *foreign_object_cloud);

		//求异物在XOY平面的中心
		float average_x = 0.0;
		float average_y = 0.0;
		for (int i = 0; i < foreign_object_cloud->size(); ++i)
		{
			average_x += foreign_object_cloud->points[i].x;
			average_y += foreign_object_cloud->points[i].y;
		}
		foreign_object.center_x = average_x / foreign_object_cloud->size();
		foreign_object.center_y = average_y / foreign_object_cloud->size();

		//剔除异物中低于轨面7cm的点，并计算最高点超出轨面高度
		for (int i = 0; i < foreign_object_cloud->size(); ++i)
		{
			float height = ComputeVerticalDistanceToPlane(foreign_object_cloud->points[i], railway_rects_);
			
			if (height > 0.07)
			{
				foreign_object.cloud->push_back(foreign_object_cloud->points[i]);
				foreign_object.height = foreign_object.height > height ? foreign_object.height : height;
			}
		}
		
		//计算异物水平投影面积
		GetBound(foreign_object.cloud, &foreign_object.bound);
		foreign_object.area = GetObjectArea(foreign_object.cloud);
		
		//判断异物位置，并计算与最近铁轨的距离
		GetObjectPosition(foreign_object, railway_rects_, railway_cloud);
		foreign_objects.push_back(foreign_object);
	}
	return foreign_objects;
}

void SafetyMonitoring::GetWithinCloud(CloudPtr cloud, CloudPtr within_cloud, std::vector<Point> vertexes, float distance)
{
	if (vertexes[0].x == vertexes[1].x)
	{
		float x_min = vertexes[0].x;
		float x_max = vertexes[2].x;
		float y_min = vertexes[1].y;
		float y_max = vertexes[0].y;
		for (int i = 0; i < cloud->size(); ++i)
		{
			float pt_x = cloud->points[i].x;
			float pt_y = cloud->points[i].y;
			if (pt_x > x_min && pt_x < x_max && pt_y > y_min && pt_y < y_max)
			{
				within_cloud->push_back(cloud->points[i]);
			}
		}
	}
	else
	{
		//求每条线段斜率及截距(k_12为铁轨方向)
		float k_1 = (vertexes[0].x - vertexes[1].x) / (vertexes[0].y - vertexes[1].y);
		float k_2 = (vertexes[2].x - vertexes[3].x) / (vertexes[2].y - vertexes[3].y);
		float k_3 = (vertexes[0].x - vertexes[2].x) / (vertexes[0].y - vertexes[2].y);
		float k_4 = (vertexes[1].x - vertexes[3].x) / (vertexes[1].y - vertexes[3].y);
		float k_12 = (k_1 + k_2) / 2;
		float k_34 = (k_3 + k_4) / 2;
		float b_1 = vertexes[0].x - k_12 * vertexes[0].y;
		float b_2 = vertexes[2].x - k_12 * vertexes[2].y;
		float b_3 = vertexes[2].x - k_34 * vertexes[2].y;
		float b_4 = vertexes[3].x - k_34 * vertexes[3].y;

		//判断截距大小
		float b_12_max = (b_1 > b_2) ? b_1 : b_2;
		float b_12_min = (b_1 < b_2) ? b_1 : b_2;
		float b_34_max = (b_3 > b_4) ? b_3 : b_4;
		float b_34_min = (b_3 < b_4) ? b_3 : b_4;

		//求指定延伸距离投影到坐标轴的长度（斜率为k_34,即b_34的范围延伸）
		float delta_x = vertexes[0].x - vertexes[2].x;
		float delta_y = vertexes[0].y - vertexes[2].y;
		float extend = fabs(distance*sqrt(delta_x*delta_x + delta_y*delta_y) / delta_y);

		//求位于四条直线范围内的点
		for (int i = 0; i < cloud->size(); ++i)
		{
			float temp_b_12 = cloud->points[i].x - k_12*cloud->points[i].y;
			float temp_b_34 = cloud->points[i].x - k_34*cloud->points[i].y;
			if (temp_b_12 > b_12_min-extend && temp_b_12 < b_12_max+extend && temp_b_34 > b_34_min && temp_b_34 < b_34_max)
			{
				within_cloud->push_back(cloud->points[i]);
			}
		}
	}
}

void SafetyMonitoring::GetWithinCloud(CloudPtr cloud, CloudPtr within_cloud, RailwayRect railway_rect)
{
	for (int i = 0; i < cloud->size(); ++i)
		{
			float temp_b_12 = cloud->points[i].x - railway_rect.parallel_line_para.k*cloud->points[i].y;
			float temp_b_34 = cloud->points[i].x - railway_rect.vertical_line_para.k*cloud->points[i].y;
			if (temp_b_12 > railway_rect.parallel_line_para.b_min - railway_rect.extend && temp_b_12 < railway_rect.parallel_line_para.b_max + railway_rect.extend
				&& temp_b_34 > railway_rect.vertical_line_para.b_min && temp_b_34 < railway_rect.vertical_line_para.b_max)
			{
				within_cloud->push_back(cloud->points[i]);
			}
		}
}

bool SafetyMonitoring::IsOnRailway(CloudPtr foreign_cloud, CloudPtr railway_cloud)
{
	//近邻搜索
	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(railway_cloud);
	float distance_threshold = 0.01;
	float count_threshold = 10;
	int count = 0;
	int K = 1;
	std::vector<int> neighbor_indices;
	std::vector<float> neighbor_distances;
	for (int i = 0; i < foreign_cloud->size(); ++i)
	{
		kdtree.nearestKSearch(foreign_cloud->points[i], K, neighbor_indices, neighbor_distances);
		if (neighbor_distances[0] < distance_threshold)
		{
			++count;
		}
	}
	if (count > count_threshold)
		return true;
	else
		return false;
}

void SafetyMonitoring::GetObjectPosition(ForeignObject& foreign_object, std::vector<std::vector<RailwayRect>> railway_rects,CloudPtr railway_cloud)
{
	for (int i = 0; i < railway_rects.size(); ++i)
	{
		for (int j = 0; j < railway_rects[i].size(); ++j)
		{
			if (IsInsideRect(foreign_object.center_x, foreign_object.center_y, railway_rects[i][j]))
			{
				foreign_object.position = i * 3 + 2;
			}
			else
			{
				float k = railway_rects[i][j].parallel_line_para.k;
				float b_max = railway_rects[i][j].parallel_line_para.b_max;
				float b_min = railway_rects[i][j].parallel_line_para.b_min;
	
				CloudPtr foreign_cloud(new Cloud);
				float distance_1 = ComputeDistanceFromCloudToLine(foreign_object.cloud, k, b_min);
				float distance_2 = ComputeDistanceFromCloudToLine(foreign_object.cloud, k, b_max);

				if (distance_1 < distance_2)
				{
					foreign_object.position = i * 3 + 1;
					foreign_object.distance = distance_1;
				}
				else
				{
					foreign_object.position = i * 3 + 3;
					foreign_object.distance = distance_2;
				}
			}
		}
	}
}

float SafetyMonitoring::GetObjectArea(CloudPtr cloud, float resolution /*= 0.01*/)
{
	Bound* bound = new Bound;
	GetBound(cloud, bound);
	
	//定义格网数组
	int row = ceil((bound->max_x - bound->min_x) / resolution);
	int list = ceil((bound->max_y - bound->min_y) / resolution);
	const int voxel_count = row * list;
	bool* grid = new bool[voxel_count];

	//判断栅格是否被点占据
	for (int i = 0; i < cloud->size(); ++i)
	{
		int pt_row = floor((cloud->points[i].x - bound->min_x) / resolution);
		int pt_list = floor((cloud->points[i].y - bound->min_y) / resolution);
		int pt_voxel_id = pt_list*row + pt_row;
		grid[pt_voxel_id] = true;
	}

	//计算被点占据的栅格总数
	int count = 0;
	for (int i = 0; i < voxel_count; ++i)
	{
		if (true == grid[i])
			++count;
	}
	float area = resolution*resolution*count;

	delete bound;
	delete grid;
	return area;
}

float SafetyMonitoring::GetRailwayPlaneHeight(ForeignObject foreign_object, std::vector<std::vector<RailwayRect>> railway_rects)
{
	for (int i = 0; i < railway_rects.size(); ++i)
	{
		for (int j = 0; j < railway_rects[i].size(); ++j)
		{
			if (IsInsideExtendedRect(foreign_object.center_x, foreign_object.center_y, railway_rects[i][j]))
			{
				PlanePara temp = railway_rects[i][j].plane_para;
				float z = -(temp.d + temp.a*foreign_object.center_x + temp.b*foreign_object.center_y) / temp.c;
				return z;
			}
		}
	}
}

float SafetyMonitoring::ComputeVerticalDistanceToPlane(Point pt, std::vector<std::vector<RailwayRect>> railway_rects)
{
	for (int i = 0; i < railway_rects.size(); ++i)
	{
		for (int j = 0; j < railway_rects[i].size(); ++j)
		{
			if (IsInsideExtendedRect(pt.x, pt.y, railway_rects[i][j]))
			{
				PlanePara temp = railway_rects[i][j].plane_para;
				float z = (temp.a*pt.x + temp.b*pt.y + temp.c*pt.z + temp.d) / sqrt(temp.a*temp.a + temp.b + temp.b + temp.c*temp.c);
				return z;
			}
		}
	}
}

bool SafetyMonitoring::IsInsideRect(float x, float y, RailwayRect railway_rect)
{
	float temp_b_12 = x - railway_rect.parallel_line_para.k*y;
	float temp_b_34 = x - railway_rect.vertical_line_para.k*y;
	if (temp_b_12 > railway_rect.parallel_line_para.b_min  && temp_b_12 < railway_rect.parallel_line_para.b_max
		&& temp_b_34 > railway_rect.vertical_line_para.b_min && temp_b_34 < railway_rect.vertical_line_para.b_max)
		return true;
	else
		return false;
}

bool SafetyMonitoring::IsInsideExtendedRect(float x, float y, RailwayRect railway_rect)
{
		float temp_b_12 = x - railway_rect.parallel_line_para.k*y;
		float temp_b_34 = x - railway_rect.vertical_line_para.k*y;
		if (temp_b_12 > railway_rect.parallel_line_para.b_min - railway_rect.extend && temp_b_12 < railway_rect.parallel_line_para.b_max + railway_rect.extend
			&& temp_b_34 > railway_rect.vertical_line_para.b_min && temp_b_34 < railway_rect.vertical_line_para.b_max)
			return true;
		else
			return false;
}

float SafetyMonitoring::ComputeDistanceFromPointToLine(float x, float y, float k,float b)
{
	return fabs(k*y - x + b) / sqrt(1 + k*k);
}

float SafetyMonitoring::ComputeDistanceFromCloudToLine(CloudPtr cloud, float k, float b)
{
	float min_distance = FLT_MAX;
	for (int i = 0; i < cloud->size(); ++i)
	{
		float temp_distance = ComputeDistanceFromPointToLine(cloud->points[i].x, cloud->points[i].y, k, b);
		min_distance = temp_distance < min_distance ? temp_distance : min_distance;
	}
	return min_distance;
}

void SafetyMonitoring::GetBound(CloudPtr cloud, Bound* bound)
{
	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt, max_pt);
	bound->min_x = min_pt[0];
	bound->min_y = min_pt[1];
	bound->min_z = min_pt[2];
	bound->max_x = max_pt[0];
	bound->max_y = max_pt[1];
	bound->max_z = max_pt[2];
}

void SafetyMonitoring::InitialGrid(CloudPtr ref_cloud,CloudPtr com_cloud, Voxel* ref_grid,Voxel* com_grid,float resolution,float min_x,float min_y,float max_x,float max_y,int list)
{
	for (int i = 0; i < ref_cloud->size(); ++i)
	{
		if (ref_cloud->points[i].x > max_x || ref_cloud->points[i].x < min_x || ref_cloud->points[i].y > max_y || ref_cloud->points[i].y < min_y)
			continue;
		else
		{
			//将点划分到对应格网
			int pt_row = floor((ref_cloud->points[i].y - min_y) / resolution);
			int pt_list = floor((ref_cloud->points[i].x - min_x) / resolution);
			int pt_voxel_id = pt_row*list + pt_list;
			ref_grid[pt_voxel_id].point_id.push_back(i);
			//寻找该栅格中最高点
			ref_grid[pt_voxel_id].max_z = (ref_cloud->points[i].z > ref_grid[pt_voxel_id].max_z) ? ref_cloud->points[i].z : ref_grid[pt_voxel_id].max_z;
		}
	}
	for (int i = 0; i < com_cloud->size(); ++i)
	{
		//将点划分到对应格网
		int pt_row = floor((com_cloud->points[i].y - min_y) / resolution);
		int pt_list = floor((com_cloud->points[i].x - min_x) / resolution);
		int pt_voxel_id = pt_row*list + pt_list;
		com_grid[pt_voxel_id].point_id.push_back(i);
		//寻找该栅格中最高点
		com_grid[pt_voxel_id].max_z = (com_cloud->points[i].z > com_grid[pt_voxel_id].max_z) ? com_cloud->points[i].z : com_grid[pt_voxel_id].max_z;
	}
}

void SafetyMonitoring::ComputeRailwayRectangle(Region region, std::vector<std::vector<RailwayRect>> railway_rects)
{
	for (int i = 0; i < region.corners.size(); i += 2)
	{
		std::vector<RailwayRect> single_railway_rects;
		for (int j = 0; j < region.corners[i].size() - 1; ++j)
		{
			RailwayRect railway_rect;
			Point p1 = region.corners[i][j];
			Point p2 = region.corners[i][j + 1];
			Point p3 = region.corners[i + 1][j];
			Point p4 = region.corners[i + 1][j + 1];

			//求每条线段斜率及截距(k_12为铁轨方向)
			float k_1 = (p1.x - p2.x) / (p1.y - p2.y);
			float k_2 = (p3.x - p4.x) / (p3.y - p4.y);
			float k_3 = (p1.x - p3.x) / (p1.y - p3.y);
			float k_4 = (p2.x - p4.x) / (p2.y - p4.y);
			float k_12 = (k_1 + k_2) / 2;
			float k_34 = (k_3 + k_4) / 2;
			float b_1 = p1.x - k_12 * p1.y;
			float b_2 = p3.x - k_12 * p3.y;
			float b_3 = p3.x - k_34 * p3.y;
			float b_4 = p4.x - k_34 * p4.y;
			railway_rect.parallel_line_para.k = k_12;
			railway_rect.vertical_line_para.k = k_34;

			//判断截距大小
			railway_rect.parallel_line_para.b_max = (b_1 > b_2) ? b_1 : b_2;
			railway_rect.parallel_line_para.b_min = (b_1 < b_2) ? b_1 : b_2;
			railway_rect.vertical_line_para.b_max = (b_3 > b_4) ? b_3 : b_4;
			railway_rect.vertical_line_para.b_min = (b_3 < b_4) ? b_3 : b_4;

			//求指定延伸距离投影到坐标轴的长度（斜率为k_34,即b_34的范围延伸）
			float delta_x = p1.x - p3.x;
			float delta_y = p1.y - p3.y;
			railway_rect.extend = fabs(region.distance*sqrt(delta_x*delta_x + delta_y*delta_y) / delta_y);

			//计算轨道平面
			railway_rect.plane_para.a = ((p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y));
			railway_rect.plane_para.b = ((p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z));
			railway_rect.plane_para.c = ((p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x));
			railway_rect.plane_para.d =  - (railway_rect.plane_para.a*p1.x + railway_rect.plane_para.b*p1.y + railway_rect.plane_para.c*p1.z);

			single_railway_rects.push_back(railway_rect);
		}
		railway_rects_.push_back(single_railway_rects);
	}
}