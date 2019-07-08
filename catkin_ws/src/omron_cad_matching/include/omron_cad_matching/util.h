
#ifndef	UTIL_H_
#define	UTIL_H_

#include <string>
#include <opencv2/core.hpp>

#include "mbp_api.h"
#include "cad_api.h"
#include "definition.h"

typedef struct
{
	float			ini_pose[3];
	BOOL			both_sided;
	INT32			min_lat;
	INT32			max_lat;
	INT32			min_lon;
	INT32			max_lon;
	INT32			min_roll;
	INT32			max_roll;
	INT32			min_dist;
	INT32			max_dist;
	INT32			object_id;
} ObjectConfig;

// File
void SplitFilepath(std::string &filepath, std::string &path, std::string &name, std::string &ext);

// Setting
INT32 ReadSetting(char *filepath, MBP_Setting *setting, char comment_char=';', char name_char='=');
INT32 ReadSettingYAML(char *filepath, MBP_Setting *setting);
INT32 ReadObjectConfig(char *filepath, ObjectConfig *obj_conf, char comment_char=';', char name_char='=');
INT32 ReadObjectConfigYAML(char *filepath, ObjectConfig *obj_conf);

// Make color image
void  MakeDepthImage(float *point_cloud, INT32 width, INT32 height, UINT8 *depth_image);
void  MakeDepthColorImage(float *point_cloud, INT32 min_depth, INT32 max_depth, cv::Mat &depth_color);
void  MakeDepthColorImage(float *point_cloud, cv::Mat& cv_depth_color);

// Save result
INT32 SaveResultImage(char *img_filename, char *cloud_filename, float proc_time, INT32 width, INT32 height, 
	MBP_SearchResult *search_result, CAD_ModelData *model_data, UINT16 draw_option, RECT *search_area=NULL);
INT32 SaveResultImage(char *filename, float proc_time, cv::Mat& cv_colorImage, 
	MBP_SearchResult *search_result, CAD_ModelData *model_data, UINT16 draw_option, RECT *search_area=NULL);
INT32 SaveSearchResult(char *result_filename, float proc_time, MBP_SearchResult *result);

#endif	/* UTIL_H_ */
