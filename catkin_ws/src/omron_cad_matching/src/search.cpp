
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include "omron_cad_matching/util.h"
#include "omron_cad_matching/util_cam.h"

using namespace std;

int search(
	string pcloud_filename, string image_filename, string mask_filename, 
	string setting_filename, string conf_filename, string model_filename)
{
	MBP_ERR_CODE		ret                = MBP_NORMAL;
	INT32				search_color_num   = 0;
	INT32       		icp_color_num      = 1;
	INT32				thresh_search      = 45;
	float       		thresh_depth       = -1;
	INT32       		thresh_grad        = -1;
	INT32				thresh_inlier_pc   = 30;
	float      			search_coef        = 1.0f;
	INT32				max_result_num     = 15;
	INT32				left			   = 0;
	INT32				top				   = 0;
	INT32				right			   = 5000;
	INT32				bottom			   = 5000;
	UINT16				out_imgtype        = 0x1111;
	INT32             	model_id, width, height, channel;
	ObjectConfig      	obj_conf;
	double            	time_search       = 0.0;
	MBP_Setting       	*mbp_setting      = NULL;
	MBP_SearchResult 	*search_result    = NULL;
	CAD_ModelData     	*model_data       = NULL;
	CAD_SearchInfo   	*search_info      = NULL;
	float             	*point_cloud      = NULL;
	UINT8             	*src_image        = NULL;
	UINT8             	*mask_image       = NULL;
	RECT              	search_area = { left, top, right, bottom };
	INT32             	color_num = MAX(search_color_num, icp_color_num);

	try {
		ROS_INFO("memory allocation");
		ret |= MBP_AllocSetting(&mbp_setting);
		ret |= CAD_AllocModelData(&model_data);
		ret |= MBP_AllocSearchResult(&search_result);
		ret |= CAD_AllocSearchInfo(&search_info);
		if (ret != MBP_NORMAL)   throw "memory error";

		ROS_INFO("read setting yaml");
		ret = ReadSettingYAML((char*)setting_filename.c_str(), mbp_setting);
		if (ret != MBP_NORMAL)   throw "wrong setting file format";
		ROS_INFO("read object config");
		ret = ReadObjectConfigYAML((char*)conf_filename.c_str(), &obj_conf);
		if (ret != MBP_NORMAL)   throw "wrong objconf file format";
		ROS_INFO("read model data");
		ret = CAD_LoadModelData((char*)model_filename.c_str(), model_data);
		if (ret != MBP_NORMAL)   throw "failed to load model data";
		ROS_INFO("read image");
		ret = ReadImageData((char*)image_filename.c_str(), color_num, &width, &height, &channel, &src_image);
		if (ret != MBP_NORMAL)   search_color_num = icp_color_num = color_num = 0;
		ret = ReadImageData((char*)mask_filename.c_str(), 1, &width, &height, &channel, &mask_image);
		ROS_INFO("read point cloud");
		//ret = ReadPointCloudTiff((char*)pcloud_filename.c_str(), &width, &height, &point_cloud);
		point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
		ret = ReadPointCloudBinary((char*)pcloud_filename.c_str(), mbp_setting->width, mbp_setting->height, point_cloud);
		if (ret != MBP_NORMAL && search_color_num < 10)   throw "failed to read point cloud";
		ROS_INFO("set global search param");
		ret = CAD_SetGlobalSearchParam(width, height, search_color_num, icp_color_num, thresh_depth, 
			thresh_grad, thresh_inlier_pc, search_area, max_result_num, 4, search_info);
		for (model_id = 0; model_id < 30; model_id++) {
			ROS_INFO("set object search param");
			ret = CAD_SetObjectSearchParam(model_id, obj_conf.min_lat, obj_conf.max_lat, obj_conf.min_lon, 
				obj_conf.max_lon, obj_conf.min_roll, obj_conf.max_roll, obj_conf.min_dist, 
				obj_conf.max_dist, thresh_search, search_coef, max_result_num, search_info);
		}

		// prepare model data and search info (call once)
		ROS_INFO("prepare for search");
		ret = CAD_PrepareForSearch(model_data, search_info);
		if (ret != MBP_NORMAL)   throw "error - CAD_PrepareForSearch()";
		
		// search
		auto start = std::chrono::system_clock::now();
		ROS_INFO("search");
		ret = CAD_Search(point_cloud, src_image, mask_image, model_data, search_info, search_result);
		if (ret != MBP_NORMAL)   throw "error - CAD_Search()";
		auto end = std::chrono::system_clock::now();
		auto diff = end - start;
		time_search = std::chrono::duration_cast<std::chrono::microseconds>(diff).count() * 0.000001;

		// save
		string filepath = string(pcloud_filename);
		string path, name, ext, result_filename;
		SplitFilepath(filepath, path, name, ext);
		result_filename = path + "/" + name + ".out";
		ROS_INFO("save search result");
		ret = SaveSearchResult((char*)result_filename.c_str(), (float)time_search, search_result);
		ROS_INFO("save search result image");
		ret = SaveResultImage((char*)image_filename.c_str(), (char*)pcloud_filename.c_str(), (float)time_search, width, height, search_result, model_data, out_imgtype);
	}
	catch(const char *error_message) {
		ROS_ERROR_STREAM(error_message);
		ret = 1;
	}

	// cleanup
	ret = MBP_DeleteSetting(&mbp_setting);
	ret = CAD_DeleteAllModelData(&model_data);
	ret = MBP_DeleteSearchResult(&search_result);
	ret = CAD_DeleteSearchInfo(&search_info);
	if (point_cloud != NULL)   free(point_cloud);
	if (src_image   != NULL)   free(src_image  );
	if (mask_image  != NULL)   free(mask_image );

    return ret;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "search");
	ROS_INFO("search");

	string pcloud_filename = "";
	string image_filename = "";
	string mask_filename = "";
	string setting_filename = "";
	string conf_filename = "";
	string model_filename = "";
    ros::param::get("~pcloud_filename", pcloud_filename);
    ros::param::get("~image_filename", image_filename);
    ros::param::get("~mask_filename", mask_filename);
    ros::param::get("~setting_filename", setting_filename);
    ros::param::get("~conf_filename", conf_filename);
    ros::param::get("~model_filename", model_filename);

	search(pcloud_filename, image_filename, mask_filename,
		setting_filename, conf_filename, model_filename);
	return 0;
}
