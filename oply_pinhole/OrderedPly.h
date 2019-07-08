//
// OrderedPly.h
//
#ifndef ORDERED_PLY_H
#define ORDERED_PLY_H
#include <string>
#include "opencv2/core/core.hpp"

//
// OrderedPly data structure.
//
struct OrderedPly {
  int size;			// number of points
  int last;			// last property of vertex element

  // element vertex
  cv::Mat_<float> point;	// 3D points (x,y,z)
  cv::Mat_<float> normal;	// normal vectors (nx,ny,nz)
  cv::Mat_<uchar> color;	// color vectors (red,green,blue)
  std::vector<float> texture;	// intensities (Texture32)
  std::vector<float> depth;	// depth (Depth32)

  // element camera
  cv::Vec3f view;
  cv::Vec3f x_axis;
  cv::Vec3f y_axis;
  cv::Vec3f z_axis;

  // element phoxi_frame_params
  int frame_width;
  int frame_height;
  int frame_index;
  float frame_start_time;
  float frame_duration;

  OrderedPly() : size(0), last(0), frame_width(0), frame_height(0),
    frame_index(0), frame_start_time(0), frame_duration(0) {};
};

//
// OrderedPly reader
//
class OPlyReader {
 private:
  std::string filename;
  OrderedPly& data;

 public:
  OPlyReader(std::string inputFile, OrderedPly& inputData);
};

#endif // ORDERED_PLY_H
