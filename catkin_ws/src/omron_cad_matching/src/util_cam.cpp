#include "omron_cad_matching/util_cam.h"
#include "omron_cad_matching/definition.h"

#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include "tiff.h"
#include "tiffio.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;
using namespace std;

//=========================================================
// Point cloud
//=========================================================

INT32 ReadPointCloudTiff(const char *tiff_filename, INT32 *width, INT32 *height, float **point_cloud)
{
	INT32   x, y, pos;
	float   *ptr_x, *ptr_y, *ptr_z;
	bool    cv_ret;
	std::vector<cv::Mat>  cv_inputImg;

	// read tiff file
	cv_ret = cv::imreadmulti(tiff_filename, cv_inputImg, -1);
	if (cv_ret == false) {
	 	return -1;
	}

	// copy
	*width = cv_inputImg[2].cols;
	*height = cv_inputImg[2].rows;
	*point_cloud = (float*)malloc((*width) * (*height) * 3 * sizeof(float));
	for (y = 0, pos = 0; y < *height; y++) {
		ptr_x = cv_inputImg[0].ptr<float>(y);
		ptr_y = cv_inputImg[1].ptr<float>(y);
		ptr_z = cv_inputImg[2].ptr<float>(y);
		for (x = 0; x < *width; x++, pos += 3) {
			if (ptr_z[x] < 1e-6f || isnan(ptr_z[x])) {
				(*point_cloud)[pos] = NAN_DEPTH;
				(*point_cloud)[pos + 1] = NAN_DEPTH;
				(*point_cloud)[pos + 2] = NAN_DEPTH;
			}
			else {
				(*point_cloud)[pos] = ptr_x[x];
				(*point_cloud)[pos + 1] = ptr_y[x];
				(*point_cloud)[pos + 2] = ptr_z[x];
			}
		}
	}

	return 0;
}

void SavePointCloudToTiff(float *src_pc, INT32 width, INT32 height, char *fileName)
{
	INT32  x, y, pos_src, pos_dst, y_offset, z_offset;
	TIFF   *imageTiff = NULL;
	float  *point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
	int    bitSize = 32;
	float  *ptrPageImage;

	// re-arrange point cloud data
	y_offset = width * height;
	z_offset = y_offset * 2;
#pragma omp parallel for firstprivate(x, pos_src, pos_dst)
	for (y = 0; y < height; y++) {
		pos_src = y * width * 3;
		pos_dst = y * width;
		for (x = 0; x < width; x++) {
			if (src_pc[pos_src+2] < 1e+6f) {
				point_cloud[pos_dst         ] = src_pc[pos_src  ];
				point_cloud[pos_dst+y_offset] = src_pc[pos_src+1];
				point_cloud[pos_dst+z_offset] = src_pc[pos_src+2];
			}
			else {
				point_cloud[pos_dst         ] = NAN;
				point_cloud[pos_dst+y_offset] = NAN;
				point_cloud[pos_dst+z_offset] = NAN;
			}
			pos_src += 3;
			pos_dst++;
		}
	}

	imageTiff = TIFFOpen(fileName, "w");
	if (imageTiff == NULL)   return;
	for (int page = 0; page < 3; page++) {
		ptrPageImage = point_cloud + (width * height) * page;

		TIFFSetField(imageTiff, TIFFTAG_IMAGEWIDTH, width);
		TIFFSetField(imageTiff, TIFFTAG_IMAGELENGTH, height);
		TIFFSetField(imageTiff, TIFFTAG_BITSPERSAMPLE, bitSize);
		TIFFSetField(imageTiff, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
		TIFFSetField(imageTiff, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(imageTiff, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
		TIFFSetField(imageTiff, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
		TIFFSetField(imageTiff, TIFFTAG_FILLORDER, FILLORDER_MSB2LSB);
		TIFFSetField(imageTiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(imageTiff, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
		TIFFSetField(imageTiff, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);
		TIFFSetField(imageTiff, TIFFTAG_PAGENUMBER, page, 3);

		size_t stride = width;
		for (int i = 0; i < height; i++) {
			TIFFWriteScanline(imageTiff, ptrPageImage + i * stride, i, 0);
		}
		TIFFWriteDirectory(imageTiff);

	}
	TIFFClose(imageTiff);
	if (point_cloud != NULL) free(point_cloud);
}

template <class T>
static void saveCloudAsDepthImage(const std::string& filename, pcl::PointCloud<T>& cloud)
{
    ROS_DEBUG_STREAM("save cloud to as image: " << filename);
    float min_depth =  1e+6f;
    float max_depth = -1e+6f;
    int min_val = 20;
    int max_val = 255;
    int width = cloud.width;
    int height = cloud.height;
    int count = width * height;

    // check range of the depth value
    for (int pos = 0; pos < count; ++pos) {
        float z = cloud.points[pos].z;
        if (z == std::numeric_limits<float>::quiet_NaN()) continue;
        if (z < min_depth) min_depth = z;
        if (z > max_depth) max_depth = z;
    }
    float coef = (max_val - min_val) / (max_depth - min_depth);
    ROS_DEBUG_STREAM("min_depth:" << min_depth);
    ROS_DEBUG_STREAM("max_depth:" << max_depth);
    ROS_DEBUG_STREAM("coef:" << coef);

    // create mono8 image
    cv::Mat cv_depth_image = cv::Mat::zeros(height, width, CV_8U);
    unsigned char *depth_image = cv_depth_image.data;
    int pos = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float z = cloud.points[pos].z;
            if (z != std::numeric_limits<float>::quiet_NaN()) {
                int val = (int)((float)min_val + coef * (z - min_depth) + 0.5f);
                depth_image[pos] = (unsigned char)(max_val - val + min_val);
            }
            pos++;
        }
    }
    cv::imwrite(filename, cv_depth_image);
}

INT32 GetPointCloudBinary(pcl::PointCloud<pcl::PointXYZ>& cloud, float *point_cloud) {
	int width  = cloud.width;
	int height = cloud.height;

    int k = 0;
    int pos = 0;
    float x, y, z;
    const float max_depth = 1e+6f + 1.0f;
    for (int j = 0; j < cloud.height; j++) 
    {
        for (int i = 0; i < cloud.width; i++) 
        {
            x = cloud.points[k].x;
            y = cloud.points[k].y;
            z = cloud.points[k].z;
            if (z < 0.1f || z == std::numeric_limits<float>::quiet_NaN()) {
                point_cloud[pos+0] = 0.0f;
                point_cloud[pos+1] = 0.0f;
                point_cloud[pos+2] = max_depth;
            }
            else {
                point_cloud[pos+0] = x * 1000.0f;
                point_cloud[pos+1] = y * 1000.0f;
                point_cloud[pos+2] = z * 1000.0f;
            }
            k++;
            pos+=3;
        }
    }
	return 0;
}

template <class T>
INT32 GetPointCloudBinary(pcl::PointCloud<T>& cloud, float *point_cloud)
{
	int width  = cloud.width;
	int height = cloud.height;

    int k = 0;
    int pos = 0;
    float x, y, z;
    const float max_depth = 1e+6f + 1.0f;
    for (int j = 0; j < cloud.height; j++) 
    {
        for (int i = 0; i < cloud.width; i++) 
        {
            x = cloud.points[k].x;
            y = cloud.points[k].y;
            z = cloud.points[k].z;
            if (z == std::numeric_limits<float>::quiet_NaN() || z < 0.1f) {
                point_cloud[pos+0] = 0.0f;
                point_cloud[pos+1] = 0.0f;
                point_cloud[pos+2] = max_depth;
            }
            else {
                point_cloud[pos+0] = x * 1000.0f;
                point_cloud[pos+1] = y * 1000.0f;
                point_cloud[pos+2] = z * 1000.0f;
            }
            k++;
            pos+=3;
        }
    }
	return 0;
}

INT32 GetPointCloudBinary(sensor_msgs::PointCloud2& cloud_msg, float *point_cloud)
{
	// XYZ
	if (cloud_msg.point_step == 24 || cloud_msg.point_step == 16) {	
		ROS_DEBUG("cloud is given.");
		using point_t = pcl::PointXYZ;
		using cloud_t = pcl::PointCloud<point_t>;
   	 	cloud_t::Ptr cloud(new cloud_t);
        pcl::fromROSMsg(cloud_msg, *cloud);
		return GetPointCloudBinary<point_t>(*cloud, point_cloud);
	}
	// XYZRGB
	else if (cloud_msg.point_step == 32) {
		ROS_DEBUG("registerd cloud is given.");
		using point_t = pcl::PointXYZRGB;
		using cloud_t = pcl::PointCloud<point_t>;
		cloud_t	cloud;
		pcl::fromROSMsg(cloud_msg, cloud);
		return GetPointCloudBinary<point_t>(cloud, point_cloud);
	} 
	else {
		ROS_ERROR_STREAM("Point cloud has worng format. " 
			<< "point_step = " << cloud_msg.point_step << ", "
			<< "width = " << cloud_msg.width << ", "
			<< "height = " << cloud_msg.height);
	}
}

INT32 ReadPointCloudBinary(const char *binary_filename, INT32 width, INT32 height, float *point_cloud)
{
	INT32   x, y, pos, ret;
	FILE    *fin = NULL;

	fin = fopen(binary_filename, "rb");
	if (fin == NULL) {
		printf("cannot open point cloud file - %s\n", binary_filename);
		return -1;
	}

	// note : unit of x,y,z is in mm
	ret = fread(point_cloud, sizeof(float), width * height * 3, fin);
	fclose(fin);
	for (y = 0, pos = 0; y < height; y++) {
		for (x = 0; x < width; x++, pos += 3) {
			if (point_cloud[pos+2] < 1e-6f || isnan(point_cloud[pos+2])) {
				point_cloud[pos    ] = NAN_DEPTH;
				point_cloud[pos + 1] = NAN_DEPTH;
				point_cloud[pos + 2] = NAN_DEPTH;
			}
		}
	}

	return 0;
}

INT32 SavePointCloudToBinary(float *src_pc, INT32 width, INT32 height, char *fileName)
{
	FILE *fout = NULL;
	fout = fopen(fileName, "wb");
	if (fout == NULL) {
		printf("cannot open point cloud file - %s\n", fileName);
		return -1;
	}

	fwrite(src_pc, sizeof(float), width * height * 3, fout);
	fclose(fout);
	return 0;
}

//=========================================================
// Image
//=========================================================

/// input image is color image (rgb) or mono image. 
/// color_num = 1 : output is mono image
/// color_num = 3 : output is color image (bgr)
INT32 ConvertColor(cv::Mat cv_inputImg, const INT32 color_num, cv::Mat& cv_image)
{
	if (cv_inputImg.data == NULL) {
		return 1;
	}
	if (color_num == 3 || color_num == 30) {
		if (cv_inputImg.channels() == 4) {
			cv::cvtColor(cv_inputImg, cv_image, cv::COLOR_BGRA2RGB);
		}
		else if (cv_inputImg.channels() == 3) {
			cv::cvtColor(cv_inputImg, cv_image, cv::COLOR_BGR2RGB);
		}
		else if (cv_inputImg.channels() == 1) {
			cv::cvtColor(cv_inputImg, cv_image, cv::COLOR_GRAY2RGB);
		}
		else {
			printf("wrong image format\n");
			return 1;
		}
	}
	else if (color_num == 1 || color_num == 10) {
		if (cv_inputImg.channels() == 4) {
			cv::cvtColor(cv_inputImg, cv_image, cv::COLOR_BGRA2GRAY);
		}
		else if (cv_inputImg.channels() == 3) {
			cv::cvtColor(cv_inputImg, cv_image, cv::COLOR_BGR2GRAY);
		}
		else if (cv_inputImg.channels() == 1) {
			cv_image = cv_inputImg.clone();
		}
		else {
			printf("wrong image format\n");
			return 1;
		}
	}
	else {
		printf("wrong argument - color num: %d", color_num);
		return 1;
	}
	return 0;
}

INT32 ReadImage(char *filename, const int color_num, cv::Mat& cv_image)
{
	cv::Mat cv_inputImg = cv::imread(filename, -1);	// bgr
	return ConvertColor(cv_inputImg, color_num, cv_image); // to rgb or mono
}

INT32 GetImageData(cv::Mat const & cv_inputImg, const INT32 color_num, INT32 *width, INT32 *height, INT32 *channel, UINT8 **src_img)
{
	cv::Mat	cv_srcImg;
	if (ConvertColor(cv_inputImg, color_num, cv_srcImg)) {
		return 1;
	}
	
	*width = cv_srcImg.cols;
	*height = cv_srcImg.rows;
	*channel = cv_srcImg.channels();
	*src_img = (UINT8*)malloc(sizeof(UINT8) * (*width) * (*height) * (*channel));
	memcpy(*src_img, cv_srcImg.data, sizeof(UINT8) * (*width) * (*height) * (*channel));
	return 0;
}

INT32 ReadImageData(const char *filename, const INT32 color_num, INT32 *width, INT32 *height, INT32 *channel, UINT8 **src_img)
{
	if (color_num == 0) return 0;
	cv::Mat cv_inputImg = cv::imread(filename, -1);
	return GetImageData(cv_inputImg, color_num, width, height, channel, src_img);
}
