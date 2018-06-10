#include "stdafx.h"
#include "safety_monitoring.h"
#include <pcl/io/pcd_io.h>

int main()
{
	/*����ԭʼ��������;*/
	CloudPtr ref_cloud(new Cloud);
	CloudPtr com_cloud(new Cloud);
	CloudPtr railway_cloud(new Cloud);

	std::string reference_cloud_path, compared_cloud_path,railway_cloud_path;
	
	std::cout << "�����׼���ƣ�\n";
	std::cin >> reference_cloud_path;
	if (!pcl::io::loadPCDFile(reference_cloud_path, *ref_cloud))
		std::cout << "��׼���Ƽ��سɹ�.\n\n";
	else
		std::cout << "��׼���Ƽ���ʧ��.\n\n";

	std::cout << "����ʵʱ����:\n";
	std::cin >> compared_cloud_path;
	if (!pcl::io::loadPCDFile(compared_cloud_path, *com_cloud))
		std::cout << "ʵʱ���Ƽ��سɹ�.\n\n";
	else
		std::cout << "ʵʱ���Ƽ���ʧ��.\n\n";
	
	//������������(txt�ļ�����)
	Region region;
	std::string txt_path;
	std::cout << "�����������ݣ�\n";
	std::cin >> txt_path;
 	std::fstream ifs(txt_path);
	std::string line;
	if (ifs)
	{
		while (std::getline(ifs, line))
		{
			++region.count;
		}
	}
 	ifs.clear();
 	ifs.seekg(0, std::ios::beg);
	
 	//�����ʼ�����Χ��region.vertexes������ת��region.corners(3��n��
 	float temp_data;
 	std::vector<float> temp_vector;
	
 	while (!ifs.eof())
 	{
 		ifs >> temp_data;
 		temp_vector.push_back(temp_data);
 	}
	int pt_count_per_railway = (temp_vector.size() - 8) / region.count;  //ÿ�������ϵĵ����

 	Point point;
	for (int i = 0; i < 8; i += 2)
	{
		point.x = temp_vector[i];
		point.y = temp_vector[i + 1];
		region.vertexes.push_back(point);
	}	
 	for (int i = 0; i < region.count; ++i)
 	{
 		std::vector<Point> single_railway;
		for (int j = 8; j < 8 + pt_count_per_railway; j += 3)
 		{
			point.x = temp_vector[i*pt_count_per_railway + j];
			point.y = temp_vector[i*pt_count_per_railway + j + 1];
			point.z = temp_vector[i*pt_count_per_railway + j + 2];
 			single_railway.push_back(point);
 		}
 		region.corners.push_back(single_railway);
 	}

// 	for (int i = 0; i < region.corners.size(); ++i)
// 	{
// 		for (int j = 0; j < region.corners[i].size(); ++j)
// 		{
// 			std::cout << region.corners[i][j].x << " " << region.corners[i][j].y << " " << region.corners[i][j].z << std::endl;
// 		}
// 		std::cout << std::endl << std::endl;
// 	}

	/*Ѱ�������;*/	
	SafetyMonitoring sm;
	std::vector<std::vector<RailwayRect>> raileway_rects;
	CloudPtr foreign_region(new Cloud);
	foreign_region = sm.RefineForeignRegion(ref_cloud, com_cloud,region);
	std::vector<ForeignObject> foreign_objects = sm.GetForeignObject(foreign_region,railway_cloud);
	
	/*����������ơ�ˮƽͶӰ����򼰸߶�;*/
	if (foreign_objects.size())
	{
		for (int i = 0; i < foreign_objects.size(); ++i)
		{
			std::cout << std::endl << std::endl;

			std::string ss_origin = "E:/railway_safety/output_data/foreign_object_" + std::to_string(i) + ".pcd";
			if (foreign_objects[i].origin_cloud->size() > 0)
				pcl::io::savePCDFile(ss_origin, *foreign_objects[i].origin_cloud);
			std::string ss_above = "E:/railway_safety/output_data/foreign_object_above_" + std::to_string(i) + ".pcd";
			if (foreign_objects[i].above_cloud->size() > 0)
				pcl::io::savePCDFile(ss_above, *foreign_objects[i].above_cloud);

			std::cout << "Foreign Cloud " << i << " has been saved.\n";
			std::cout << "The size of the foreign cloud is " << foreign_objects[i].origin_cloud->size() << std::endl;
			std::cout << "center_x : " << foreign_objects[i].center_x << std::endl;
			std::cout << "center_y : " << foreign_objects[i].center_y<< std::endl;
			std::cout << "area : " << foreign_objects[i].area << std::endl;
			std::cout << "Height above the railway plane is " << foreign_objects[i].height << "m.\n";
			std::cout << "The position of foreign object is " << foreign_objects[i].position << "\n";
			if (foreign_objects[i].position != 2)
				std::cout << "The distance between foreign object and railway is " << foreign_objects[i].distance << "m\n";	
		}
	}
	else
	{
		std::cout << "no change.\n";
	}

	system("pause");
	return 0;
}