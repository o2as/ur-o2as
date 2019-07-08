#include <stdlib.h>
#include <stdio.h>
#include <chrono>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "omron_cad_matching/cad_api.h"
#include "omron_cad_matching/util.h"
#include "omron_cad_matching/TrainModel.h"
#include "omron_cad_matching/SaveModel.h"

namespace o2as {
namespace omron {

using namespace std;

class TrainServerNode
{
public:
    TrainServerNode()
	{
		MBP_ERR_CODE ret = MBP_NORMAL;
		ret |= CAD_AllocModelData(&model_data);
		if (ret != MBP_NORMAL) throw "memory error";

		// activate service servers
		servers_.init_model = this->nh.advertiseService("init_model", &TrainServerNode::initModelCallback, this);
		servers_.train_model = this->nh.advertiseService("train_model", &TrainServerNode::trainModelCallback, this);
		servers_.save_model = this->nh.advertiseService("save_model", &TrainServerNode::saveModelCallback, this);
	}

    virtual ~TrainServerNode()
	{
		MBP_ERR_CODE ret = MBP_NORMAL;
		ret = CAD_DeleteAllModelData(&model_data);
	}

	bool initModel()
	{
		MBP_ERR_CODE ret = MBP_NORMAL;
		try {
			ret |= CAD_DeleteAllModelData(&model_data);
			ret |= CAD_AllocModelData(&model_data);
			return true;
		}
		catch(const char *error_message) {
			ROS_ERROR("%s", error_message);
		}
		return false;
	}

    bool initModelCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
	{
		return initModel();
	}

    bool trainModelCallback(
        omron_cad_matching::TrainModel::Request &req,
        omron_cad_matching::TrainModel::Response &res)
	{
		ROS_INFO("requested: Train(cad=%s)", (char*)req.cad_filename.c_str());

		INT32  model_id = -1;
		double time_train = 0.0;
		MBP_ERR_CODE ret = MBP_NORMAL;
		try {
			// allocate memory
			ret |= MBP_AllocSetting(&mbp_setting);
			ret |= MBP_AllocMesh(&mesh_obj);
			ret |= MBP_AllocMesh(&mesh_train);
			ret |= MBP_AllocMeshes(&mesh_weight);
			ret |= CAD_AllocTrainParam(&train_param);
			if (ret != MBP_NORMAL) throw "memory error";

			// camera setting
			memset(mbp_setting->cam_param, 0, 9 * sizeof(float));
			mbp_setting->cam_param[8] 	= 1.0f;
			mbp_setting->width         	= req.camera_setting.width;
			mbp_setting->height        	= req.camera_setting.height;
			mbp_setting->cam_param[0]  	= req.camera_setting.focal_length_x;
			mbp_setting->cam_param[4]  	= req.camera_setting.focal_length_y;
			mbp_setting->cam_param[2]  	= req.camera_setting.principal_point_x;
			mbp_setting->cam_param[5]  	= req.camera_setting.principal_point_y;
			mbp_setting->dist_param[0] 	= req.camera_setting.dist_param_k1;
			mbp_setting->dist_param[1] 	= req.camera_setting.dist_param_k2;
			mbp_setting->dist_param[2] 	= req.camera_setting.dist_param_p1;
			mbp_setting->dist_param[3] 	= req.camera_setting.dist_param_p2;
			mbp_setting->dist_param[4] 	= req.camera_setting.dist_param_k3;

			// read mesh data from STL file
			ret = MBP_ReadSTL((char*)req.cad_filename.c_str(), mesh_obj);
			if (ret != MBP_NORMAL)   throw "failed to read stl file";

			// set training parameter
			float ini_pose[3];
			ini_pose[0] 		= req.train_setting.ini_pose_x;
			ini_pose[1] 		= req.train_setting.ini_pose_y;
			ini_pose[2] 		= req.train_setting.ini_pose_z;
			INT32 object_id 	= req.train_setting.object_id;
			BOOL both_sided 	= req.train_setting.both_sided;
			INT32 min_lat 		= req.train_setting.min_lat;
			INT32 max_lat 		= req.train_setting.max_lat;
			INT32 min_lon 		= req.train_setting.min_lon;
			INT32 max_lon 		= req.train_setting.max_lon;
			INT32 min_dist 		= req.train_setting.min_dist;
			INT32 max_dist 		= req.train_setting.max_dist;
			INT32 thread_num 	= req.train_setting.thread_num;
			ret = CAD_SetTrainParam(mbp_setting, object_id, both_sided, min_lat, max_lat, min_lon, max_lon, min_dist, max_dist, ini_pose, thread_num, train_param);

			// train
			auto start = std::chrono::system_clock::now();
			ret = CAD_Train(train_param, mesh_obj, mesh_train, mesh_weight, model_data, &model_id);
			if (ret != MBP_NORMAL)   throw "error - CAD_Train()";
			auto end = std::chrono::system_clock::now();
			auto diff = end - start;
			time_train = std::chrono::duration_cast<std::chrono::microseconds>(diff).count() * 0.000001;
			time_train_total += time_train;
			res.success = true;
		}
		catch(const char *error_message) {
			ROS_ERROR("%s", error_message);
			ROS_ERROR("error code = %d", ret);
			res.success = false;
		}

		// cleanup
		CAD_DeleteTrainParam(&train_param);
		MBP_DeleteMesh(&mesh_obj);
		MBP_DeleteMesh(&mesh_train);
		MBP_DeleteAllMeshes(&mesh_weight);
		MBP_DeleteSetting(&mbp_setting);

		// set response
		res.model_id = model_id;
		return res.success;
	}

    bool saveModelCallback(
        omron_cad_matching::SaveModel::Request &req,
        omron_cad_matching::SaveModel::Response &res)
	{
		ROS_INFO("execute save model");
		ROS_INFO("data_filename: %s", (char*)req.data_filename.c_str());
		ROS_INFO("text_filename: %s", (char*)req.text_filename.c_str());
		MBP_ERR_CODE ret = MBP_NORMAL;
		try {
			// save
			ret = CAD_SaveModelData((char*)req.data_filename.c_str(), model_data);
			if (ret != MBP_NORMAL)   throw "error - CAD_SaveModelData()";
			ret = CAD_SaveModelDataTxt((char*)req.text_filename.c_str(), time_train_total, model_data);
			if (ret != MBP_NORMAL)   throw "error - CAD_SaveModelDataTxt()";
			res.success = true;
		}
		catch(const char *error_message) {
			ROS_ERROR("%s", error_message);
			ROS_DEBUG("code = %d", ret);
			res.success = false;
		}
		return res.success;
	}

    void run();

protected:
    ros::NodeHandle nh;

	MBP_Setting    	*mbp_setting      = NULL;
	CAD_TrainParam 	*train_param      = NULL;
	CAD_ModelData  	*model_data       = NULL;
	MBP_Mesh       	*mesh_obj         = NULL;
	MBP_Mesh       	*mesh_train       = NULL;
	MBP_Meshes     	*mesh_weight      = NULL;
    double         	time_train_total  = 0.0;

	struct Servers {
		ros::ServiceServer init_model;
		ros::ServiceServer train_model;
		ros::ServiceServer save_model;
	} servers_;
};

} // namespace omron
} // namespace o2as

using namespace o2as::omron;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cad_matching_train_server");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
    TrainServerNode node;
	ros::spin();
    return 0;
}
