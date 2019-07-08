#include <chrono>
#include <string>
#include <ros/ros.h>
#include "omron_cad_matching/util.h"

using namespace std;

int train(string cad_filename, string conf_filename, string setting_filename, string model_filename)
{
	// filepath
	string path, name, ext;
	SplitFilepath(model_filename, path, name, ext);
	string train_filename = path + "/" + name + "_train.txt";
	ROS_INFO("cad_filename = %s", cad_filename.c_str());
	ROS_INFO("conf_filename = %s", conf_filename.c_str());
	ROS_INFO("setting_filename = %s", setting_filename.c_str());
	ROS_INFO("model_filename = %s", model_filename.c_str());
	ROS_INFO("train_filename = %s", train_filename.c_str());

	MBP_ERR_CODE      ret;
	INT32             model_id;
	ObjectConfig      obj_conf;
	double            time_train = 0.0;
	MBP_Setting       *mbp_setting      = NULL;
	CAD_TrainParam    *train_param      = NULL;
	CAD_ModelData     *model_data       = NULL;
	MBP_Mesh          *mesh_obj         = NULL;
	MBP_Mesh          *mesh_train       = NULL;
	MBP_Meshes        *mesh_weight      = NULL;

	// allocation
	try {
		ret |= MBP_AllocSetting(&mbp_setting);
		ret |= CAD_AllocModelData(&model_data);
		ret |= CAD_AllocTrainParam(&train_param);
		ret |= MBP_AllocMesh(&mesh_obj);
		ret |= MBP_AllocMesh(&mesh_train);
		ret |= MBP_AllocMeshes(&mesh_weight);
		if (ret != MBP_NORMAL)   throw "memory error";

		// set data
		ROS_INFO("read setting yaml");
		ret = ReadSettingYAML((char*)setting_filename.c_str(), mbp_setting);
		if (ret != MBP_NORMAL)   throw "wrong setting file format";
		ROS_INFO("read object config yaml");
		ret = ReadObjectConfigYAML((char*)conf_filename.c_str(), &obj_conf);
		if (ret != MBP_NORMAL)   throw "wrong objconf file format";
		ROS_INFO("read stl");
		ret = MBP_ReadSTL((char*)cad_filename.c_str(), mesh_obj);
		if (ret != MBP_NORMAL)   throw "failed to read stl file";

		// parameter
		ROS_INFO("set train param");
		ret = CAD_SetTrainParam(mbp_setting, obj_conf.object_id, obj_conf.both_sided, obj_conf.min_lat, obj_conf.max_lat,
				obj_conf.min_lon, obj_conf.max_lon, obj_conf.min_dist, obj_conf.max_dist, obj_conf.ini_pose, 4, train_param);

		// training
		auto start = std::chrono::system_clock::now();
		ROS_INFO("train");
		ret = CAD_Train(train_param, mesh_obj, mesh_train, mesh_weight, model_data, &model_id);
		if (ret != MBP_NORMAL)   throw "error - CAD_Train()";
		auto end = std::chrono::system_clock::now();
		auto diff = end - start;
		time_train = std::chrono::duration_cast<std::chrono::microseconds>(diff).count() * 0.000001;

		// save
		ROS_INFO("save model data");
		ret = CAD_SaveModelData((char*)model_filename.c_str(), model_data);
		if (ret != MBP_NORMAL)   throw "error - CAD_SaveModelData()";
		ROS_INFO("save model data txt");
		ret = CAD_SaveModelDataTxt((char*)train_filename.c_str(), time_train, model_data);
		if (ret != MBP_NORMAL)   throw "error - CAD_SaveModelDataTxt()";
	}
	catch(const char *error_message) {
		cout << error_message << endl;
		ret = 1;
	}
	
	// delete
	ret = CAD_DeleteTrainParam(&train_param);
	ret = MBP_DeleteSetting(&mbp_setting);
	ret = MBP_DeleteMesh(&mesh_obj);
	ret = MBP_DeleteMesh(&mesh_train);
	ret = MBP_DeleteAllMeshes(&mesh_weight);
	ret = CAD_DeleteAllModelData(&model_data);

    return ret;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "train_single");
	ROS_INFO("train_single");

	string cad_filename = "";
	string conf_filename = "";
	string setting_filename = "";
	string model_filename = "";
    ros::param::get("~cad_filename", cad_filename);
    ros::param::get("~conf_filename", conf_filename);
    ros::param::get("~setting_filename", setting_filename);
    ros::param::get("~model_filename", model_filename);

	train(cad_filename, conf_filename, setting_filename, model_filename);
	return 0;
}
