// std
#include <stdlib.h>
#include <stdio.h>

// cad
#include "omron_cad_matching/win_type.h"
#include "omron_cad_matching/cad_api.h"
#include "omron_cad_matching/util.h"
#include "omron_cad_matching/util_cam.h"

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// ros
#include <ros/ros.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <omron_cad_matching/SearchConfig.h>

// message
#include <geometry_msgs/Point.h>
#include "omron_cad_matching/LoadModelData.h"
#include "omron_cad_matching/ValidateModelData.h"
#include "omron_cad_matching/InvalidateModelData.h"
#include "omron_cad_matching/Search.h"
#include "omron_cad_matching/Search2.h"
#include "omron_cad_matching/SearchResult.h"
#include <omron_cad_matching/DetectedObject.h>
#include <omron_cad_matching/SearchResult.h>
#include <omron_cad_matching/GlobalSearchParam.h>
#include <omron_cad_matching/ObjectSearchParam.h>

namespace o2as {
namespace omron {

using namespace std;

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

class SearchServerNode
{
protected:
    // data
	MBP_SearchResult *search_result_  = NULL;
	CAD_ModelData    *model_data_     = NULL;
	CAD_SearchInfo   *search_info_    = NULL;

    // search params
    bool model_changed_               = false;
    bool model_loaded_                = false;
    bool search_parameter_changed_    = false;
    bool prepared_                    = false;
    bool display_                     = true;

    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<omron_cad_matching::SearchConfig> dynamic_reconfigure_server_;
    const uint32_t set_default_dynamic_reconfig_values = 0xffffffff;
    std::string image_dir_            = "";

	struct Servers {
        ros::ServiceServer load_model_data;
        ros::ServiceServer validate_model_data;
        ros::ServiceServer invalidate_model_data;
        ros::ServiceServer search;
        ros::ServiceServer search2;
	} servers_;

public:
    SearchServerNode() : nh_("~")
    {
        // memory allocation
        allocate_memory();

        // reconfigure
        ros::param::get("image_dir", image_dir_);
        dynamic_reconfigure::Server<omron_cad_matching::SearchConfig>::CallbackType f;
        f = boost::bind(&SearchServerNode::dynamic_reconfigure_callback, this, _1, _2);
        dynamic_reconfigure_server_.setCallback(f);

        // rosservice
        servers_.load_model_data         = nh_.advertiseService("load_model_data",         &SearchServerNode::executeLoadModelData,        this);
        servers_.validate_model_data     = nh_.advertiseService("validate_model_data",     &SearchServerNode::executeValidateModelData,    this);
        servers_.invalidate_model_data   = nh_.advertiseService("invalidate_model_data",   &SearchServerNode::executeInvalidateModelData,  this);
        servers_.search                  = nh_.advertiseService("search",                  &SearchServerNode::executeSearch,               this);
        servers_.search2                 = nh_.advertiseService("search2",                 &SearchServerNode::executeSearch2,               this);

        ROS_DEBUG("search server is ready.");
    }

    virtual ~SearchServerNode()
    {
        delete_memory();
    }

protected:

    //----------------------------------
    // dynamic reconfigure
    //----------------------------------

    enum search_param{
        param_width = 0,
        param_height,
        param_search_color_num,
        param_icp_color_num,
        param_thresh_depth,
        param_thresh_grad,
        param_thresh_inlier_pc,
        param_search_area_left,
        param_search_area_top,
        param_search_area_right,
        param_search_area_bottom,
        param_max_result_num_all,
        param_thread_num,
        param_min_lat,
        param_max_lat,
        param_min_lon,
        param_max_lon,
        param_min_roll,
        param_max_roll,
        param_min_dist,
        param_max_dist,
        param_thresh_search,
        param_search_coef,
        param_max_result_num,
        param_display,
        param_count
    };

    template<class T>
    static bool ros_param_set(const char* name, T& value)
    {
        bool changed = false;
        if (ros::param::has(name) == false) {
            ros::param::set(name, value);
            changed = true;
        }
        else {
            T current;
            ros::param::get(name, current);
            if (value != current) {
                ros::param::set(name, value);
                changed = true;
            }
        }
        return changed;
    }

    void dynamic_reconfigure_callback(omron_cad_matching::SearchConfig &config, uint32_t level)
    {
        ROS_DEBUG_STREAM("search parameter is changed by dynamic reconfigure: " << level);

        if (level & (1 << param_width)) {
            ROS_DEBUG_STREAM("width: " << config.width);
            search_parameter_changed_ = ros_param_set<int>("~width" , config.width);
        }
        if (level & (1 << param_width)) {
            ROS_DEBUG_STREAM("height: " << config.height);
            search_parameter_changed_ = ros_param_set<int>("~height" , config.height);
        }
        if (level & (1 << param_search_color_num)) {
            ROS_DEBUG_STREAM("search_color_num: " << config.search_color_num);
            search_parameter_changed_ = ros_param_set<int>("~search_color_num" , config.search_color_num);
        }
        if (level & (1 << param_icp_color_num)) {
            ROS_DEBUG_STREAM("icp_color_num: " << config.icp_color_num);
            search_parameter_changed_ = ros_param_set<int>("~icp_color_num" , config.icp_color_num);
        }
        if (level & (1 << param_thresh_depth)) {
            ROS_DEBUG_STREAM("thresh_depth: " << config.thresh_depth);
            search_parameter_changed_ = ros_param_set<double>("~thresh_depth" , config.thresh_depth);
        }
        if (level & (1 << param_thresh_grad)) {
            ROS_DEBUG_STREAM("thresh_grad: " << config.thresh_grad);
            search_parameter_changed_ = ros_param_set<int>("~thresh_grad" , config.thresh_grad);
        }
        if (level & (1 << param_thresh_inlier_pc)) {
            ROS_DEBUG_STREAM("thresh_inlier_pc: " << config.thresh_inlier_pc);
            search_parameter_changed_ = ros_param_set<int>("~thresh_inlier_pc" , config.thresh_inlier_pc);
        }
        if (level & (1 << param_search_area_left)) {
            ROS_DEBUG_STREAM("search_area_left: " << config.search_area_left);
            search_parameter_changed_ = ros_param_set<int>("~search_area_left" , config.search_area_left);
        }
        if (level & (1 << param_search_area_top)) {
            ROS_DEBUG_STREAM("search_area_top: " << config.search_area_top);
            search_parameter_changed_ = ros_param_set<int>("~search_area_top" , config.search_area_top);
        }
        if (level & (1 << param_search_area_right)) {
            ROS_DEBUG_STREAM("search_area_right: " << config.search_area_right);
            search_parameter_changed_ = ros_param_set<int>("~search_area_right" , config.search_area_right);
        }
        if (level & (1 << param_search_area_bottom)) {
            ROS_DEBUG_STREAM("search_area_bottom: " << config.search_area_bottom);
            search_parameter_changed_ = ros_param_set<int>("~search_area_bottom" , config.search_area_bottom);
        }
        if (level & (1 << param_max_result_num_all)) {
            ROS_DEBUG_STREAM("max_result_num_all: " << config.max_result_num_all);
            search_parameter_changed_ = ros_param_set<int>("~max_result_num_all" , config.max_result_num_all);
        }
        if (level & (1 << param_thread_num)) {
            ROS_DEBUG_STREAM("thread_num: " << config.thread_num);
            search_parameter_changed_ = ros_param_set<int>("~thread_num" , config.thread_num);
        }
        if (level & (1 << param_min_lat)) {
            ROS_DEBUG_STREAM("min_lat: " << config.min_lat);
            search_parameter_changed_ = ros_param_set<int>("~min_lat" , config.min_lat);
        }
        if (level & (1 << param_max_lat)) {
            ROS_DEBUG_STREAM("max_lat: " << config.max_lat);
            search_parameter_changed_ = ros_param_set<int>("~max_lat" , config.max_lat);
        }
        if (level & (1 << param_min_lon)) {
            ROS_DEBUG_STREAM("min_lon: " << config.min_lon);
            search_parameter_changed_ = ros_param_set<int>("~min_lon" , config.min_lon);
        }
        if (level & (1 << param_max_lon)) {
            ROS_DEBUG_STREAM("max_lon: " << config.max_lon);
            search_parameter_changed_ = ros_param_set<int>("~max_lon" , config.max_lon);
        }
        if (level & (1 << param_min_roll)) {
            ROS_DEBUG_STREAM("min_roll: " << config.min_roll);
            search_parameter_changed_ = ros_param_set<int>("~min_roll" , config.min_roll);
        }
        if (level & (1 << param_max_roll)) {
            ROS_DEBUG_STREAM("max_roll: " << config.max_roll);
            search_parameter_changed_ = ros_param_set<int>("~max_roll" , config.max_roll);
        }
        if (level & (1 << param_min_dist)) {
            ROS_DEBUG_STREAM("min_dist: " << config.min_dist);
            search_parameter_changed_ = ros_param_set<int>("~min_dist" , config.min_dist);
        }
        if (level & (1 << param_max_dist)) {
            ROS_DEBUG_STREAM("max_dist: " << config.max_dist);
            search_parameter_changed_ = ros_param_set<int>("~max_dist" , config.max_dist);
        }
        if (level & (1 << param_thresh_search)) {
            ROS_DEBUG_STREAM("thresh_search: " << config.thresh_search);
            search_parameter_changed_ = ros_param_set<int>("~thresh_search" , config.thresh_search);
        }
        if (level & (1 << param_search_coef)) {
            ROS_DEBUG_STREAM("search_coef: " << config.search_coef);
            search_parameter_changed_ = ros_param_set<double>("~search_coef" , config.search_coef);
        }
        if (level & (1 << param_max_result_num)) {
            ROS_DEBUG_STREAM("max_result_num: " << config.max_result_num);
            search_parameter_changed_ = ros_param_set<int>("~max_result_num" , config.max_result_num);
        }
        if (level & (1 << param_display)) {
            ROS_DEBUG_STREAM("display: " << config.display);
            display_ = config.display;
            if (display_ == false) {
                cvDestroyAllWindows();
            }
        }
    }

    //----------------------------------
    // memory
    //----------------------------------

    void allocate_memory()
    {
        MBP_ERR_CODE ret = MBP_NORMAL;
        ret |= CAD_AllocModelData(&model_data_);
        ret |= CAD_AllocSearchInfo(&search_info_);
        if (ret != MBP_NORMAL) throw "memory error";
    }

    void delete_memory()
    {
        MBP_ERR_CODE ret = MBP_NORMAL;
        ret = CAD_DeleteAllModelData(&model_data_);
        ret = CAD_DeleteSearchInfo(&search_info_);
    }

    void reallocate_memory()
    {
        model_loaded_ = false;
        prepared_ = false;
        search_parameter_changed_ = true;
        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            delete_memory();
            allocate_memory();
        }
        catch(const char *error_message) {
            ROS_ERROR("%s", error_message);
        }
    }

    bool reallocate_model_data()
    {
        model_loaded_ = false;
        prepared_ = false;
        search_parameter_changed_ = true;
        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            ret |= CAD_DeleteAllModelData(&model_data_);
            ret |= CAD_AllocModelData(&model_data_);
            return true;
        }
        catch(const char *error_message) {
            return false;
        }
    }

    bool reallocate_search_info()
    {
        prepared_ = false;
        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            ret |= CAD_DeleteSearchInfo(&search_info_);
            ret |= CAD_AllocSearchInfo(&search_info_);
            return true;
        }
        catch(const char *error_message) {
            return false;
        }
    }

    //----------------------------------
    // load model
    //----------------------------------

    bool loadModelData(std::string model_filename)
    {
        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            ROS_INFO_STREAM("Load cad matching model data" << model_filename);
            ret = CAD_LoadModelData(model_filename.c_str(), model_data_);
            if (ret != MBP_NORMAL) throw "failed to load model data";
            model_loaded_ = true;
            model_changed_ = false;
        }
        catch(const char *error_message) {
            ROS_ERROR("%s", error_message);
            ROS_ERROR("error = %d", ret);
            return false;
        }
        return true;
    }

    bool executeLoadModelData(
        omron_cad_matching::LoadModelData::Request &req,
        omron_cad_matching::LoadModelData::Response &res)
    {
        try {
            if (!req.model_filename.empty()){
                ros::param::set("~model_filename", req.model_filename);
                model_changed_ = true;
                res.success = true;
            }
        }
        catch(const char *error_message) {
            res.success = false;
            return false;
        }
        return true;
    }

    //----------------------------------
    // validate / invalidate model
    //----------------------------------

    bool executeValidateModelData(
        omron_cad_matching::ValidateModelData::Request &req,
        omron_cad_matching::ValidateModelData::Response &res)
    {
        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            int n = req.model_ids.size();
            for (int i=0; i<n; i++) {
                UINT8 model_id = req.model_ids[i];
                ROS_INFO_STREAM("Cad matching model data " << model_id << " was validated.");
                ret = CAD_ValidateModelData(model_id, model_data_);
            }
            res.success = true;
        }
        catch(const char *error_message) {
            ROS_ERROR("%s", error_message);
            res.success = false;
            return false;
        }
        return true;
    }

    bool executeInvalidateModelData(
        omron_cad_matching::InvalidateModelData::Request &req,
        omron_cad_matching::InvalidateModelData::Response &res)
    {
        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            int n = req.model_ids.size();
            for (int i=0; i<n; i++) {
                UINT8 model_id = req.model_ids[i];
                ROS_INFO_STREAM("Cad matching model data " << model_id << " was invalidated.");
                ret = CAD_InvalidateModelData(model_id, model_data_);
            }
            res.success = true;
        }
        catch(const char *error_message) {
            ROS_INFO("%s", error_message);
            res.success = false;
            return false;
        }
        return true;
    }

    //----------------------------------
    // global search param
    //----------------------------------

    void debugDumpGlobalSearchParam(const omron_cad_matching::GlobalSearchParam& param)
    {
        ROS_DEBUG("global_search_param.width = %d", param.width); 
        ROS_DEBUG("global_search_param.height = %d", param.height); 
        ROS_DEBUG("global_search_param.search_color_num = %d", param.search_color_num); 
        ROS_DEBUG("global_search_param.icp_color_num = %d", param.icp_color_num); 
        ROS_DEBUG("global_search_param.thresh_depth = %f", param.thresh_depth); 
        ROS_DEBUG("global_search_param.thresh_grad = %d", param.thresh_grad); 
        ROS_DEBUG("global_search_param.thresh_inlier_pc = %d", param.thresh_inlier_pc); 
        ROS_DEBUG("global_search_param.search_area = { %d, %d, %d, %d }", 
            (int)param.search_area_left, (int)param.search_area_top,
            (int)param.search_area_right, (int)param.search_area_bottom); 
        ROS_DEBUG("global_search_param.max_result_num_all = %d", param.max_result_num_all); 
        ROS_DEBUG("global_search_param.thread_num = %d", param.thread_num);
    }

    void getGlobalSearchParamFromROSParam(omron_cad_matching::GlobalSearchParam& param)
    {
        ros::param::get("~width", param.width);
        ros::param::get("~height", param.height);
        ros::param::get("~search_color_num", param.search_color_num);
        ros::param::get("~icp_color_num", param.icp_color_num);
        ros::param::get("~thresh_depth", param.thresh_depth);
        ros::param::get("~thresh_grad", param.thresh_grad);
        ros::param::get("~thresh_inlier_pc", param.thresh_inlier_pc);
        ros::param::get("~search_area_left", param.search_area_left);
        ros::param::get("~search_area_top", param.search_area_top);
        ros::param::get("~search_area_right", param.search_area_right);
        ros::param::get("~search_area_bottom", param.search_area_bottom);
        ros::param::get("~max_result_num_all", param.max_result_num_all);
        ros::param::get("~thread_num", param.thread_num);
    }

    //----------------------------------
    // object search param
    //----------------------------------

    void debugDumpObjectSearchParam(const omron_cad_matching::ObjectSearchParam& param)
    {
        ROS_DEBUG("min_lat = %d", param.min_lat); 
        ROS_DEBUG("max_lat = %d", param.max_lat); 
        ROS_DEBUG("min_lon = %d", param.min_lon); 
        ROS_DEBUG("max_lon = %d", param.max_lon); 
        ROS_DEBUG("min_roll = %d", param.min_roll); 
        ROS_DEBUG("max_roll = %d", param.max_roll); 
        ROS_DEBUG("min_dist = %d", param.min_dist); 
        ROS_DEBUG("max_dist = %d", param.max_dist); 
        ROS_DEBUG("thresh_search = %d", param.thresh_search); 
        ROS_DEBUG("search_coef = %f", param.search_coef); 
        ROS_DEBUG("max_result_num = %d", param.max_result_num);
    }

    void getObjectSearchParamFromROSParam(omron_cad_matching::ObjectSearchParam& param)
    {
        ros::param::get("~min_lat", param.min_lat);
        ros::param::get("~max_lat", param.max_lat);
        ros::param::get("~min_lon", param.min_lon);
        ros::param::get("~max_lon", param.max_lon);
        ros::param::get("~min_roll", param.min_roll);
        ros::param::get("~max_roll", param.max_roll);
        ros::param::get("~min_dist", param.min_dist);
        ros::param::get("~max_dist", param.max_dist);
        ros::param::get("~thresh_search", param.thresh_search);
        ros::param::get("~search_coef", param.search_coef);
        ros::param::get("~max_result_num", param.max_result_num);
    }

    //----------------------------------
    // prepare
    //----------------------------------

    bool prepare(
        const omron_cad_matching::GlobalSearchParam& global_search_param, 
        const omron_cad_matching::ObjectSearchParam& object_search_param)
    {
        ROS_DEBUG("prepare to search object");
        debugDumpGlobalSearchParam(global_search_param);
        debugDumpObjectSearchParam(object_search_param);
        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            // gloabl search param
            RECT search_area = { global_search_param.search_area_left, global_search_param.search_area_top, global_search_param.search_area_right, global_search_param.search_area_bottom };
            ret = CAD_SetGlobalSearchParam(
                global_search_param.width, global_search_param.height, global_search_param.search_color_num, global_search_param.icp_color_num, 
                global_search_param.thresh_depth, global_search_param.thresh_grad, global_search_param.thresh_inlier_pc, search_area, 
                global_search_param.max_result_num_all, global_search_param.thread_num, search_info_);
            if (ret != MBP_NORMAL) throw "failed to set global search param";

            // object search param
            for (INT32 model_id = 0; model_id < 30; model_id++)
            {
                ret = CAD_SetObjectSearchParam(model_id,
                    object_search_param.min_lat, object_search_param.max_lat, object_search_param.min_lon, object_search_param.max_lon, 
                    object_search_param.min_roll, object_search_param.max_roll, object_search_param.min_dist, object_search_param.max_dist, 
                    object_search_param.thresh_search, object_search_param.search_coef, object_search_param.max_result_num, search_info_);
                if (ret != MBP_NORMAL) throw "failed to set object search param";
            }

            // prepare
            ret = CAD_PrepareForSearch(model_data_, search_info_);
            if (ret != MBP_NORMAL) throw "failed to prepare for search";
        }
        catch(const char *error_message) {
            ROS_ERROR("%s", error_message);
            ROS_ERROR("error = %d", ret);
            prepared_ = false;
            return false;
        }
        search_parameter_changed_ = false;
        prepared_ = true;
        return true;
    }

    //----------------------------------
    // search
    //----------------------------------

    void printSearchResult(MBP_SearchResult *result)
    {
        // print search result
        if (result->result_num == 0) {
            ROS_INFO("-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\n");
        }
        else {
            for (int i = 0; i < result->result_num; i++) {
                ROS_INFO("id=%2d, pos=(%5.5f, %5.5f, %5.5f), rot=(%5.5f, %5.5f, %5.5f), score=(%5.5f, %5.5f, %5.5f), id=%3d", i, 
                    result->pos3D[i][0], result->pos3D[i][1], result->pos3D[i][2], result->rot3D[i][0], result->rot3D[i][1], result->rot3D[i][2], 
                    result->score2D[i], result->score3D[i], result->score_final[i], result->object_id[i]);
            }
        }
    }

    bool display(int width, int height, float* point_cloud, cv::Mat& cv_image, MBP_SearchResult *result, std::string result_filename)
    {
        float       disp_scale 		    = 2;
        INT32       disp_w              = (INT32)ROUND(disp_scale * (float)960);
        INT32       disp_h              = (INT32)ROUND(disp_scale * (float)360);
        INT32       input_w             = disp_w / 3;
        INT32       input_h             = disp_h / 2;
        cv::Mat     cv_disp_image       = cv::Mat(disp_h, disp_w, CV_8UC3);
        cv::Rect    cv_roi_color        = cv::Rect(0, 0, input_w, input_h);
        cv::Rect    cv_roi_depth        = cv::Rect(0, input_h, input_w, input_h);
        cv::Rect    cv_roi_result       = cv::Rect(input_w, 0, disp_w - input_w, disp_h);
        cv::Mat     roi_color (cv_disp_image, cv_roi_color );
        cv::Mat     roi_depth (cv_disp_image, cv_roi_depth );
        cv::Mat     roi_result(cv_disp_image, cv_roi_result);

        cv::Mat cv_color_image  = cv_image.clone();
        cv::Mat cv_depth_color  = cv::Mat(height, width, CV_8UC3);
        cv::Mat cv_result_image = cv::imread((char*)result_filename.c_str(), -1);
        MakeDepthColorImage(point_cloud, 200, 350, cv_depth_color);
        cv::resize(cv_result_image, roi_result, roi_result.size());
        cv::resize(cv_color_image, roi_color, roi_color.size());
        cv::resize(cv_depth_color, roi_depth, roi_depth.size());
        cv::imshow("6D pose estimation", cv_disp_image);
        cv::waitKey(1);
    }
    
    omron_cad_matching::SearchResult toSearchResultMsg(MBP_SearchResult *result)
    {
        omron_cad_matching::SearchResult msg;
        msg.result_num = result->result_num;
        if (result->result_num > 0) {
            for (int i = 0; i < result->result_num; i++) {
                omron_cad_matching::DetectedObject obj;
                obj.object_id   = result->object_id[i];
                obj.model_id    = result->model_id[i];
                obj.score2D     = result->score2D[i];
                obj.score3D     = result->score3D[i];
                obj.score_final = result->score_final[i];
                obj.valid_flag  = result->valid_flag[i];
                obj.pos3D.x     = result->pos3D[i][0];
                obj.pos3D.y     = result->pos3D[i][1];
                obj.pos3D.z     = result->pos3D[i][2];
                obj.rot3D.resize(3);
                obj.rot3D[0]    = result->rot3D[i][0];
                obj.rot3D[1]    = result->rot3D[i][1];
                obj.rot3D[2]    = result->rot3D[i][2];
                obj.rot_mat.resize(9);
                for (int j=0; j<9; j++) {
                    obj.rot_mat[j] = result->rot_mat[i][j];
                }
                msg.detected_objects.push_back(obj);
            }
        }
        return msg;
    }

    template <class T>
    bool search(
        float *point_cloud,
        UINT8 *color_image,
        UINT8 *mask_image,
        omron_cad_matching::GlobalSearchParam& global_search_param,
        T &res,
        std::string result_filename, 
        cv::Mat& result_image,
        cv::Mat& result_depth)
    {
        ROS_DEBUG("cad_matching: search service is called.");

        // set result filename
        std::string path, name, ext;
        SplitFilepath(result_filename, path, name, ext);
        std::string image_filename = path + "/" + name + ".jpg";
        std::string depth_filename = path + "/" + name + "_depth.jpg";

        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            if (search_parameter_changed_ == true || model_changed_ == true) {
                ROS_DEBUG("search setting is changed");
                reallocate_memory();
            }
            if (model_loaded_ == false) {
                ROS_DEBUG("model is not loaded");
                std::string model_filename = "";
                ros::param::get("~model_filename", model_filename);
                loadModelData(model_filename);
            }

            if (prepared_ == false) {
                ROS_DEBUG("cad matching is not prepared.");
                omron_cad_matching::ObjectSearchParam object_search_param;
                getObjectSearchParamFromROSParam(object_search_param);
                prepare(global_search_param, object_search_param);
            }

            // allocate
            ret |= MBP_AllocSearchResult(&search_result_);
            if (ret != MBP_NORMAL) throw "memory allocation error";

            // search
            ROS_DEBUG_STREAM("cad search");
            ret = CAD_Search(point_cloud, color_image, mask_image, model_data_, search_info_, search_result_);

            // save result
            ROS_DEBUG_STREAM("save search result");
            ret = SaveSearchResult((char*)result_filename.c_str(), (float)0, search_result_);
            if (color_image != NULL) {
                int left, top, right, bottom;
                ros::param::get("~search_area_left", left);
                ros::param::get("~search_area_top", top);
                ros::param::get("~search_area_right", right);
                ros::param::get("~search_area_bottom", bottom);
                RECT search_area = { 
                    global_search_param.search_area_left, global_search_param.search_area_top,
                    global_search_param.search_area_right, global_search_param.search_area_bottom };
                ret = SaveResultImage((char*)image_filename.c_str(), 0, result_image,
                    search_result_, model_data_, 0x1111, &search_area);
                ret = SaveResultImage((char*)depth_filename.c_str(), 0, result_depth,
                    search_result_, model_data_, 0x1111, &search_area);
            }

            // display result
            ROS_DEBUG_STREAM("display search result");
            if (display_ == true) {
                display(global_search_param.width, global_search_param.height, point_cloud, 
                    result_image, search_result_, image_filename);
            }

            // copy search result to ROS data structure
            printSearchResult(search_result_);
            res.search_result = toSearchResultMsg(search_result_);
            res.success = true;
        }
        catch(const char *error_message) {
            ROS_ERROR("%s", error_message);
            res.success = false;
        }

        // cleanup
        MBP_DeleteSearchResult(&search_result_);

        ROS_DEBUG("SearchServerNode::executeSearch() end");
        return res.success;
    }

    bool executeSearch(
        omron_cad_matching::Search::Request &req,
        omron_cad_matching::Search::Response &res)
    {
        cv::Mat cv_result_image;
        cv::Mat cv_result_depth;
        float *point_cloud = NULL;
        UINT8 *color_image = NULL;
        UINT8 *mask_image  = NULL;

        omron_cad_matching::GlobalSearchParam global_search_param;
        getGlobalSearchParamFromROSParam(global_search_param);
        INT32 width = global_search_param.width;
        INT32 height = global_search_param.height;
        INT32 color_num = std::max(global_search_param.search_color_num, global_search_param.icp_color_num);

        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            // get input data (file or memory)
            INT32 channel = 0;
            std::string pcloud_filename = req.pcloud_filename;
            std::string image_filename = req.image_filename;
            std::string mask_filename = req.mask_filename;

            // read point cloud from the file
            if (!pcloud_filename.empty()) {
                ROS_DEBUG_STREAM("read cloud data: " << pcloud_filename);
                point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
                ret = ReadPointCloudBinary((char*)pcloud_filename.c_str(), width, height, point_cloud);
                if (ret != MBP_NORMAL) throw "failed to read point cloud binary.";
                cv_result_depth = cv::Mat::zeros(height, width, CV_8UC3);
           		MakeDepthColorImage(point_cloud, cv_result_depth);
            }
            else {
                throw "point cloud filename is not valid.";
            }

            // read image from the file
            if (!image_filename.empty()) {
                ROS_DEBUG_STREAM("read image: " << image_filename);
                cv_result_image = cv::imread((char*)image_filename.c_str(), -1); // bgr or mono
                if (cv_result_image.channels()==1) {
                    cv::Mat cv_image;
                    cv::equalizeHist(cv_result_image, cv_image);
                    cv::cvtColor(cv_image, cv_result_image, cv::COLOR_GRAY2BGR);         
                }
                ret = GetImageData(cv_result_image, color_num, &width, &height, &channel, &color_image); // rgb or mono
                if (ret != MBP_NORMAL && color_num) throw "failed to read image file.";
            } 

            // read mask from the file
            if (!mask_filename.empty()) {
                ROS_DEBUG_STREAM("read mask: " << mask_filename);
                ret = ReadImageData((char*)mask_filename.c_str(), 1, &width, &height, &channel, &mask_image);
                if (ret != MBP_NORMAL) throw "failed to read mask image file.";
            }

            // set result filename
            std::string path, name, ext;
            SplitFilepath(pcloud_filename, path, name, ext);
            std::string result_filename = path + "/" + name + ".out";
            res.success = search<omron_cad_matching::Search::Response>(
                point_cloud, color_image, mask_image, global_search_param, res, result_filename, cv_result_image, cv_result_depth);
        }
        catch(const char *error_message) {
            ROS_ERROR("%s", error_message);
            res.success = false;
        }

        if (point_cloud != NULL) free(point_cloud);
        if (color_image != NULL) free(color_image);
        if (mask_image != NULL) free(mask_image);
        return res.success;
    }

    bool executeSearch2(
        omron_cad_matching::Search2::Request &req,
        omron_cad_matching::Search2::Response &res)
    {
        cv::Mat cv_result_image;
        cv::Mat cv_result_depth;
        float *point_cloud = NULL;
        UINT8 *color_image = NULL;
        UINT8 *mask_image  = NULL;

        omron_cad_matching::GlobalSearchParam global_search_param;
        getGlobalSearchParamFromROSParam(global_search_param);
        INT32 width = global_search_param.width;
        INT32 height = global_search_param.height;
        INT32 color_num = std::max(global_search_param.search_color_num, global_search_param.icp_color_num);

        MBP_ERR_CODE ret = MBP_NORMAL;
        try {
            INT32 channel = 0;
            std::string result_filename = image_dir_ + "/cad_matching.out";

            // get point cloud data from the ros message
            if (req.pcloud.width > 0 && req.pcloud.height) {
                ROS_DEBUG("pcloud : width = %d, height = %d", req.pcloud.width, req.pcloud.height);
                point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
                ret = GetPointCloudBinary(req.pcloud, point_cloud);
                if (ret != MBP_NORMAL) throw "failed to get point cloud binary";
                cv_result_depth = cv::Mat::zeros(height, width, CV_8UC3);
           		MakeDepthColorImage(point_cloud, cv_result_depth);
            }

            // get image data from the ros message
            if (req.image.width > 0 && req.image.height > 0) {
                ROS_DEBUG_STREAM("image: " << req.image.width << ", " << req.image.height << ", " << req.image.encoding);
                cv::Mat cv_image = cv_bridge::toCvCopy(req.image, req.image.encoding)->image;
                if (cv_image.channels()==1) {
                    cv::Mat cv_image_tmp;
                    cv::equalizeHist(cv_image, cv_image_tmp);
                    cv::cvtColor(cv_image_tmp, cv_result_image, cv::COLOR_GRAY2BGR);         
                }
                ret = GetImageData(cv_image, color_num, &width, &height, &channel, &color_image);
                if (ret != MBP_NORMAL && color_num) throw "failed to get image data.";
            }

            // get mask image data from the ros message
            if (req.mask.width > 0 && req.mask.height > 0) {
                ROS_DEBUG_STREAM("mask: " << req.image.width << ", " << req.image.height << ", " << req.image.encoding);
                cv::Mat cv_mask = cv_bridge::toCvCopy(req.mask, req.mask.encoding)->image;
                ret = GetImageData(cv_mask, 1, &width, &height, &channel, &mask_image);
                if (ret != MBP_NORMAL) throw "failed to get mask image file.";
            }

            res.success = search<omron_cad_matching::Search2::Response>(
                point_cloud, color_image, mask_image, global_search_param, res, result_filename, cv_result_image, cv_result_depth);
        }
        catch(const char *error_message) {
            ROS_ERROR("%s", error_message);
            res.success = false;
        }

        if (point_cloud != NULL) free(point_cloud);
        if (color_image != NULL) free(color_image);
        if (mask_image != NULL) free(mask_image);
        return res.success;
    }
};

} // namespace omron
} // namespace o2as

using namespace o2as::omron;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omron_cad_matching_search_server");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
    SearchServerNode node;
    ros::spin();
    return 0;
}
