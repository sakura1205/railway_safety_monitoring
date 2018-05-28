#include "stdafx.h"
#include "safety_monitoring.h"
#include <pcl/io/pcd_io.h>

int main()
{
	/*读入原始点云数据;*/
	CloudPtr ref_cloud(new Cloud);
	CloudPtr com_cloud(new Cloud);

	std::string reference_cloud_path, compared_cloud_path;
	
	std::cout << "输入基准点云：\n";
	std::cin >> reference_cloud_path;
	if (!pcl::io::loadPCDFile(reference_cloud_path, *ref_cloud))
		std::cout << "基准点云加载成功.\n\n";
	else
		std::cout << "基准点云加载失败.\n\n";

	std::cout << "输入实时点云:\n";
	std::cin >> compared_cloud_path;
	if (!pcl::io::loadPCDFile(compared_cloud_path, *com_cloud))
		std::cout << "实时点云加载成功.\n\n";
	else
		std::cout << "实时点云加载失败.\n\n";

	float x_1 = 8.60978, y_1 = 1.55716;
	float x_2 = 11.7662, y_2 = 1.60929;
	float x_3 = 13.1423, y_3 = 1.4075;
	float x_4 = 9.88809, y_4 = 1.50581;
	float nx = -0.02700101, ny = -0.99931675;
	float vertical_x1 = 13.137211, vertical_y1 = 2.707609;
	float vertical_x2 = 13.144015, vertical_y2 = 0.666721;
	float parallel_distance = 0.35;
	float vertical_distance = 0.1;

	RailwayParameters rp;
	rp.parallel_distance = parallel_distance;
	rp.vertical_distance = vertical_distance;
	rp.parallel_x.push_back(x_1);
	rp.parallel_x.push_back(x_2);
	rp.parallel_x.push_back(x_3);
	rp.parallel_x.push_back(x_4);
	rp.parallel_y.push_back(y_1);
	rp.parallel_y.push_back(y_2);
	rp.parallel_y.push_back(y_3);
	rp.parallel_y.push_back(y_4);
	rp.vertical_x1 = vertical_x1;
	rp.vertical_y1 = vertical_y1;
	rp.vertical_x2 = vertical_x2;
	rp.vertical_y2 = vertical_y2;
	rp.nx = nx;
	rp.ny = ny;


	/*寻找异物点;*/
	SafetyMonitoring sm;
	CloudPtr foreign_region(new Cloud);
	foreign_region = sm.RefineForeignRegion(ref_cloud, com_cloud,rp);
	std::vector<ForeignObject> foreign_objects = sm.GetForeignObject(foreign_region);
	
	/*输出异物点点云、水平投影外包框及高度;*/
	if (foreign_objects.size())
	{
		for (int i = 0; i < foreign_objects.size(); ++i)
		{
			std::cout << std::endl << std::endl;
			std::string ss = "E:/railway_safety/output_data/foreign_object_" + std::to_string(i) + ".pcd";
			pcl::io::savePCDFile(ss, *foreign_objects[i].cloud);
			std::cout << "Foreign Cloud " << i << " has been saved.\n";
			std::cout << "The size of the foreign cloud is " << foreign_objects[i].cloud->size() << std::endl;
			std::cout << "min_x : " << foreign_objects[i].bound.min_x << std::endl;
			std::cout << "max_x : " << foreign_objects[i].bound.max_x << std::endl;
			std::cout << "min_y : " << foreign_objects[i].bound.min_y << std::endl;
			std::cout << "max_y : " << foreign_objects[i].bound.max_y << std::endl;
			std::cout << "Height of the foreign cloud is " << foreign_objects[i].height << "m.\n";
		}
	}
	else
	{
		std::cout << "no change.\n";
	}

	system("pause");
	return 0;
}