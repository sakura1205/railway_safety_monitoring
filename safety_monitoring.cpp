#include "stdafx.h"
#include "safety_monitoring.h"

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>



SafetyMonitoring::SafetyMonitoring()
{
	initial_resolution_ = 1;
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

CloudPtr SafetyMonitoring::RefineForeignRegion(CloudPtr ref_cloud, CloudPtr com_cloud,RailwayParameters rp)
{
	//截取待检测范围内的点云
	CloudPtr ref_within_cloud(new Cloud);
	CloudPtr com_within_cloud(new Cloud);
	GetWithinCloud(ref_cloud, ref_within_cloud,rp);
	GetWithinCloud(com_cloud, com_within_cloud, rp);
// 	pcl::io::savePCDFile("E:/com_within_cloud.pcd", *com_within_cloud);
// 	pcl::io::savePCDFile("E:/ref_within_cloud.pcd", *ref_within_cloud);

	//寻找异物区域
	for (float resolution = initial_resolution_; resolution > final_resolution_; resolution /= 2)
	{
		CloudPtr ref_foreign_region(new Cloud);
		CloudPtr com_foreign_region(new Cloud);
		FindForeignRegion(ref_within_cloud, com_within_cloud,resolution,ref_foreign_region,com_foreign_region);
		ref_within_cloud = ref_foreign_region;
 		com_within_cloud = com_foreign_region;
	}
	std::cout << "foreign region size : " << com_cloud->size() << std::endl;
	pcl::io::savePCDFile("E:/foreign_region.pcd", *com_within_cloud);
	return com_within_cloud;
}

std::vector<ForeignObject> SafetyMonitoring::GetForeignObject(CloudPtr foreign_region)
{
	std::vector<ForeignObject> foreign_objects;
	//获取单个异物点索引
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(foreign_region);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	CloudPtr foreign_objects_cloud(new Cloud);
	ec.setClusterTolerance(0.2); // 20cm
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
		pcl::copyPointCloud(*foreign_object_cloud, *foreign_object.cloud);
		GetBound(foreign_object.cloud, &foreign_object.bound);
		foreign_object.height = foreign_object.bound.max_z - foreign_object.bound.min_z;
		foreign_objects.push_back(foreign_object);
	}
	return foreign_objects;
}

void SafetyMonitoring::GetWithinCloud(CloudPtr cloud, CloudPtr within_cloud, RailwayParameters rp)
{
	//求铁轨条数
	int count = rp.parallel_x.size();

	//求各铁轨斜率及截距
	float k_1 = rp.ny / rp.nx;
	std::vector<float> y_b;
	for (int i = 0; i < count; ++i)
	{
		float temp_b = rp.parallel_y[i] - k_1*rp.parallel_x[i];
		y_b.push_back(temp_b);
	}
	std::sort(y_b.begin(), y_b.end());

	//求两垂直线斜率及与x轴交点
	float k_2 = -rp.nx / rp.ny;
	std::vector<float> x_intercept;
	float temp_intercept = -(rp.vertical_y1 - k_2*rp.vertical_x1)/k_2;
	x_intercept.push_back(temp_intercept);
	temp_intercept = -(rp.vertical_y2 - k_2*rp.vertical_x2)/k_2;
	x_intercept.push_back(temp_intercept);
	std::sort(x_intercept.begin(), x_intercept.end());

	//将延伸距离归化到x、y轴
	float y_distance = fabs((rp.parallel_distance*sqrt(rp.nx*rp.nx + rp.ny*rp.ny)) / rp.nx);
	float x_distance = fabs((rp.vertical_distance*sqrt(rp.nx*rp.nx + rp.ny*rp.ny)) / rp.nx);

	//求位于范围内的点云
	for (int i = 0; i < cloud->size(); ++i)
	{
		float pt_b = cloud->points[i].y - k_2*cloud->points[i].x;
		float pt_intercept = -pt_b / k_2;
		if (pt_intercept > x_intercept[0]-x_distance && pt_intercept < x_intercept[1]+x_distance)
		{
			float pt_b_1 = cloud->points[i].y - k_1*cloud->points[i].x;
			for (int j = 0; j < count; j += 2)
			{			
				if (pt_b_1 > y_b[j]-y_distance && pt_b_1 < y_b[j+1]+y_distance)
				{
					within_cloud->push_back(cloud->points[i]);
					break;
				}
				else
					continue;
			}
		}
	}
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
