#include "omron_cad_matching/util.h"
#include "omron_cad_matching/util_cam.h"

#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <limits>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

//=========================================================
// File
//=========================================================

inline void DeleteSpace(string &buf)
{
    size_t pos;
    while((pos = buf.find_first_of(" @\t")) != string::npos){
        buf.erase(pos, 1);
    }
}

void SplitFilepath(string &filepath, string &path, string &name, string &ext)
{
	path = filepath;
	int   pos = path.find_last_of('.');
	ext = path.substr(pos);
	path.erase(path.begin()+pos, path.end());
	pos = path.find_last_of("\\/");
	name = path.substr(pos + 1);
	path.erase(path.begin()+pos, path.end());
}

//=========================================================
// Setting
//=========================================================

INT32 ReadSetting(char *filepath, MBP_Setting *setting, char comment_char, char name_char)
{
    std::ifstream   ifs(filepath);
    std::string     str, name;
	int             check_sum, i;
	int             flag[SETTING_PARAM_NUM] = {0};
    if (ifs.fail()) {
        std::cerr << "cannot open file" << filepath << std::endl;
        return -1;
    }

	memset(setting->cam_param, 0, 9 * sizeof(float));
    setting->cam_param[8] = 1.0f;
    while (getline(ifs, str)) {
        int pos_comment = str.find(comment_char);
        if (pos_comment > 0)   str.erase(str.begin()+pos_comment, str.end());
        int pos_name    = str.find(name_char);
        name = str.substr(0, pos_name);
        if (pos_name    > 0)   str.erase(str.begin(), str.begin()+pos_name+1);
        DeleteSpace(name);
        if (name == string("width"))               setting->width         = atoi((const char*)str.c_str()), flag[ 0] = 1;
        if (name == string("height"))              setting->height        = atoi((const char*)str.c_str()), flag[ 1] = 1;
        if (name == string("focal_length_x"))      setting->cam_param[0]  = atof((const char*)str.c_str()), flag[ 2] = 1;
        if (name == string("focal_length_y"))      setting->cam_param[4]  = atof((const char*)str.c_str()), flag[ 3] = 1;
        if (name == string("principal_point_x"))   setting->cam_param[2]  = atof((const char*)str.c_str()), flag[ 4] = 1;
        if (name == string("principal_point_y"))   setting->cam_param[5]  = atof((const char*)str.c_str()), flag[ 5] = 1;
        if (name == string("dist_param_k1"))       setting->dist_param[0] = atof((const char*)str.c_str()), flag[ 6] = 1;
        if (name == string("dist_param_k2"))       setting->dist_param[1] = atof((const char*)str.c_str()), flag[ 7] = 1;
        if (name == string("dist_param_p1"))       setting->dist_param[2] = atof((const char*)str.c_str()), flag[ 8] = 1;
        if (name == string("dist_param_p2"))       setting->dist_param[3] = atof((const char*)str.c_str()), flag[ 9] = 1;
        if (name == string("dist_param_k3"))       setting->dist_param[4] = atof((const char*)str.c_str()), flag[10] = 1;
    }
	check_sum = 0;
	for (i = 0; i < SETTING_PARAM_NUM; i++)   check_sum += flag[i];
	if (check_sum != SETTING_PARAM_NUM)   return 1;
    //cout << setting->width << setting->height << setting->cam_param[5] << setting->dist_param[4] << endl;

    return 0;
}
INT32 ReadSettingYAML(char *filepath, MBP_Setting *setting)
{
	return ReadSetting(filepath, setting, '#', ':');
}

INT32 ReadObjectConfig(char *filepath, ObjectConfig *obj_conf, char comment_char, char name_char)
{
    std::ifstream   ifs(filepath);
    std::string     str, name;
	int             check_sum, i;
	int             flag[OBJCONF_PARAM_NUM] = {0};
    if (ifs.fail()) {
        std::cerr << "cannot open file" << filepath << std::endl;
        return -1;
    }

    while (getline(ifs, str)) {
        int pos_comment = str.find(comment_char);
        if (pos_comment > 0)   str.erase(str.begin()+pos_comment, str.end());
        int pos_name    = str.find(name_char);
        name = str.substr(0, pos_name);
        if (pos_name    > 0)   str.erase(str.begin(), str.begin()+pos_name+1);
        DeleteSpace(name);
        if (name == string("initial_pose_x" ))   obj_conf->ini_pose[0]  = atof((const char*)str.c_str()), flag[ 0] = 1;
        if (name == string("initial_pose_y" ))   obj_conf->ini_pose[1]  = atof((const char*)str.c_str()), flag[ 1] = 1;
        if (name == string("initial_pose_z" ))   obj_conf->ini_pose[2]  = atof((const char*)str.c_str()), flag[ 2] = 1;
        if (name == string("both_side"      ))   obj_conf->both_sided   = atoi((const char*)str.c_str()), flag[ 3] = 1;
        if (name == string("min_latitude"   ))   obj_conf->min_lat      = atoi((const char*)str.c_str()), flag[ 4] = 1;
        if (name == string("max_latitude"   ))   obj_conf->max_lat      = atoi((const char*)str.c_str()), flag[ 5] = 1;
        if (name == string("max_longitude"  ))   obj_conf->max_lon      = atoi((const char*)str.c_str()), flag[ 6] = 1;
        if (name == string("min_longitude"  ))   obj_conf->min_lon      = atoi((const char*)str.c_str()), flag[ 7] = 1;
        if (name == string("min_camera_roll"))   obj_conf->min_roll     = atoi((const char*)str.c_str()), flag[ 8] = 1;
        if (name == string("max_camera_roll"))   obj_conf->max_roll     = atoi((const char*)str.c_str()), flag[ 9] = 1;
        if (name == string("min_distance"   ))   obj_conf->min_dist     = atoi((const char*)str.c_str()), flag[10] = 1;
        if (name == string("max_distance"   ))   obj_conf->max_dist     = atoi((const char*)str.c_str()), flag[11] = 1;
        if (name == string("object_id"      ))   obj_conf->object_id    = atoi((const char*)str.c_str()), flag[12] = 1;
    }
	check_sum = 0;
	for (i = 0; i < OBJCONF_PARAM_NUM; i++)   check_sum += flag[i];
	if (check_sum != OBJCONF_PARAM_NUM)   return 1;
    //cout << obj_conf->ini_pose[1] << " " << obj_conf->max_lon << " " << obj_conf->object_id << endl;

    return 0;
}
INT32 ReadObjectConfigYAML(char *filepath, ObjectConfig *obj_conf)
{
	return ReadObjectConfig(filepath, obj_conf, '#', ':');
}

//=========================================================
// Depth
//=========================================================

void MakeDepthImage(float *point_cloud, INT32 width, INT32 height, UINT8 *depth_image)
{
	INT32 pos, depth_pos, val;
	float min_depth, max_depth, coef;
	INT32 pix_num;
	INT32 min_val = 20;
	INT32 max_val = 255;

	memset(depth_image, 0, width * height * sizeof(UINT8));
	pix_num = width * height * 3;
	min_depth =  1e+6f;
	max_depth = -1e+6f;
	for (pos = 0; pos < pix_num; pos+=3) {
		if (point_cloud[pos+2] < MAX_DEPTH) {
			if (point_cloud[pos+2] < min_depth)   min_depth = point_cloud[pos+2];
			if (point_cloud[pos+2] > max_depth)   max_depth = point_cloud[pos+2];
		}
	}

	coef = (max_val - min_val) / (max_depth - min_depth);

	for (pos = 0, depth_pos = 0; pos < pix_num; pos+=3, depth_pos++) {
		if (point_cloud[pos+2] < MAX_DEPTH) {
			val = (INT32)((float)min_val + coef * (point_cloud[pos+2] - min_depth) + 0.5f);
			depth_image[depth_pos] = (UINT8)(max_val - val + min_val);
		}
	}
}

void MakeDepthColorImage(float *point_cloud, INT32 min_depth, INT32 max_depth, cv::Mat &cv_depth_color)
{
	INT32   y, x, pos;
	UINT8   *ptr_gray;
	float   val, coef, denom;
	INT32   width  = cv_depth_color.cols;
	INT32   height = cv_depth_color.rows;
	UINT8   min_val = 50;
	cv::Mat depth_gray = cv::Mat(height, width, CV_8UC1);
	
	// gray depth
	denom = (float)(max_depth - min_depth);
	coef = (255.9f - min_val) / denom;

#pragma omp parallel for firstprivate(x, pos, ptr_gray, val)
	for (y = 0; y < height; y++) {
		pos = y * width * 3;
		ptr_gray  = depth_gray.ptr<UINT8>(y);
		for (x = 0; x < width; x++) {
			val = (point_cloud[pos + 2] - (float)min_depth) * coef + (float)min_val;
			ptr_gray[x] = MIN((UINT8)ROUND(val), 255);
			pos += 3;
		}
	}

	cv::applyColorMap(depth_gray, cv_depth_color, cv::COLORMAP_JET);
}

void MakeDepthColorImage(float *point_cloud, cv::Mat& cv_depth_color)
{
	INT32 width  = cv_depth_color.cols;
	INT32 height = cv_depth_color.rows;
	cv::Mat cv_depthImg = cv::Mat::zeros(height, width, CV_8U);

	UINT8 *depth_image = NULL;
	depth_image = (UINT8*)malloc(width * height * sizeof(UINT8));
	if (depth_image == NULL)   goto EXCEPTION;
	MakeDepthImage(point_cloud, width, height, depth_image);

	memcpy(cv_depthImg.data, depth_image, sizeof(UINT8) * width * height);
	cv::applyColorMap(cv_depthImg, cv_depth_color, cv::COLORMAP_BONE);
	
EXCEPTION:
	if (depth_image != NULL)   free(depth_image);
}

//=========================================================
// Save result
//=========================================================

INT32 SaveResultImage(char *filename, float proc_time, cv::Mat& cv_colorImage,
	MBP_SearchResult *search_result, CAD_ModelData *model_data, UINT16 draw_option, RECT *search_area)
{
	if (draw_option == 0)   return 0;
	if (cv_colorImage.rows == 0 || cv_colorImage.cols == 0) return -1;

	UINT8 *edge_image = NULL;
	INT32 ret, x, y, pos, result_id;
	INT32 width = cv_colorImage.cols;
	INT32 height = cv_colorImage.rows;

	vector<int>  jpeg_params(2);
	jpeg_params[0] = cv::IMWRITE_JPEG_QUALITY;
	jpeg_params[1] = 95;

	// draw search area
	if (search_area != NULL) {
		cv::rectangle(cv_colorImage, cv::Rect(cv::Point(search_area->left, search_area->top), cv::Point(search_area->right, search_area->bottom)), cv::Scalar(255,0,0), 1, cv::LINE_AA);
	}

	// draw contour
	if ((draw_option & 0x0001) > 0) {
		edge_image = (UINT8*)malloc(width * height * sizeof(UINT8));
		ret = CAD_DrawResultImage(search_result, model_data, width, height, FALSE, edge_image, NULL);
		for (y = 0, pos = 0; y < height; y++) {
			for (x = 0; x < width; x++, pos++) {
				if (edge_image[pos] > 0) {
					cv::Point	pt_tmp(x, y);
					cv::circle(cv_colorImage, pt_tmp, 1, cv::Scalar(0,255,0), -1, cv::LINE_AA, 0);
				}
			}
		}
	}

	for (result_id = 0; result_id < search_result->result_num; result_id++) {
		// axes
		if ((draw_option & 0x0100) > 0) {
			cv::Point  pt_org, pt_x, pt_y, pt_z;

			pt_org.x = (INT32)ROUND(search_result->axis[result_id][0]);
			pt_org.y = (INT32)ROUND(search_result->axis[result_id][1]);
			pt_x  .x = (INT32)ROUND(search_result->axis[result_id][2]);
			pt_x  .y = (INT32)ROUND(search_result->axis[result_id][3]);
			pt_y  .x = (INT32)ROUND(search_result->axis[result_id][4]);
			pt_y  .y = (INT32)ROUND(search_result->axis[result_id][5]);
			pt_z  .x = (INT32)ROUND(search_result->axis[result_id][6]);
			pt_z  .y = (INT32)ROUND(search_result->axis[result_id][7]);

			cv::arrowedLine(cv_colorImage, pt_org, pt_x, cv::Scalar(255, 255,   0), 2, cv::LINE_AA);
			cv::arrowedLine(cv_colorImage, pt_org, pt_y, cv::Scalar(  0, 255, 255), 2, cv::LINE_AA);
			cv::arrowedLine(cv_colorImage, pt_org, pt_z, cv::Scalar(  0, 128, 255), 2, cv::LINE_AA);
		}
	}

	// time
	if ((draw_option & 0x1000) > 0) {
		char   time_char[256];
		sprintf(time_char, "%8.1f ms", proc_time * 1000.0);
		cv::putText(cv_colorImage, time_char, cv::Point(2, 20), cv::FONT_HERSHEY_PLAIN, 1.25, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
	}

	// save
	cv::imwrite(filename, cv_colorImage, jpeg_params);

	if (edge_image != NULL) free(edge_image);
}

INT32 SaveResultImage(char *img_filename, char *cloud_filename, float proc_time, INT32 width, INT32 height, 
	MBP_SearchResult *search_result, CAD_ModelData *model_data, UINT16 draw_option, RECT *search_area)
{
	INT32 ret;
	string path, name, ext;
	string filepath = string(cloud_filename);
	SplitFilepath(filepath, path, name, ext);
	string result_filename = path + "/" + name + ".jpg";

	float* point_cloud = NULL;
	cv::Mat cv_colorImage;
	ret = ReadImage(img_filename, 3, cv_colorImage);
	if (ret == 1) {
	    point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
    	ret = ReadPointCloudBinary(cloud_filename, width, height, point_cloud);
		if (ret != 0) goto EXCEPTION;
		MakeDepthColorImage(point_cloud, cv_colorImage);
	}

	SaveResultImage((char*)result_filename.c_str(), proc_time, cv_colorImage, search_result, model_data, draw_option, search_area);

EXCEPTION:
	if (point_cloud != NULL)   free(point_cloud);
}

INT32 SaveSearchResult(char *result_filename, float proc_time, MBP_SearchResult *result)
{
	FILE *fout = fopen(result_filename, "wt");
	if (fout == NULL) {
		printf("file open error - %s", result_filename);
		return 1;
	}

	if (result->result_num == 0) {
		fprintf(fout, "-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t-1\t%15.7f\n", proc_time);
	}
	else {
		for (INT32 i = 0; i < result->result_num; i++) {
			fprintf(fout, "%2d\t%15.7f\t%15.7f\t%15.7f\t%15.7f\t%15.7f\t%15.7f\t%15.7f\t%15.7f\t%15.7f\t%3d\t%15.7f\n", 
				i, result->pos3D[i][0], result->pos3D[i][1], result->pos3D[i][2], result->rot3D[i][0], result->rot3D[i][1], result->rot3D[i][2], 
				result->score2D[i], result->score3D[i], result->score_final[i], result->object_id[i], proc_time);
		}
	}

	fclose(fout);
	return 0;
}
