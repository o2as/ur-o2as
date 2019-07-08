
#pragma once

#include "win_type.h"
#include <opencv2/core.hpp>
#include <sensor_msgs/PointCloud2.h>

// Save and read poinc cloud data
INT32 ReadPointCloudTiff(const char *tiff_filename, INT32 *width, INT32 *height, float **point_cloud);
void  SavePointCloudToTiff(float *src_pc, INT32 width, INT32 height, char *fileName);
INT32 GetPointCloudBinary(sensor_msgs::PointCloud2& cloud_msg, float *point_cloud);
INT32 ReadPointCloudBinary(const char *binary_filename, INT32 width, INT32 height, float *point_cloud);
INT32 SavePointCloudToBinary(float *src_pc, INT32 width, INT32 height, char *fileName);

// Save and read image data
INT32 GetImageData(cv::Mat const & cv_inputImg, const INT32 color_num, INT32 *width, INT32 *height, INT32 *channel, UINT8 **src_img);
INT32 ReadImageData(const char *filename, const INT32 color_num, INT32 *width, INT32 *height, INT32 *channel, UINT8 **src_img);
INT32 ReadImage(char *filename, const int color_num, cv::Mat& cv_image);
