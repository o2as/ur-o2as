/*
  oply_pinhole.cpp

  Copyright (C) 2018 AIST, All rights reserved.

  Any using, copying, disclosing information regarding the software and
  documentation without permission of the copyright holders are prohibited.
  The software is provided "AS IS", without warranty of any kind, express or
  implied, including all implied warranties of merchantability and fitness.
  In no event shall the authors or copyright holders be liable for any claim,
  damages or other liability, whether in an action of contract, tort or 
  otherwise, arising from, out of or in connection with the software or 
  the use or other dealings in the software.
*/
#include <iostream>
#include <fstream>
#include <iomanip>
#include <exception>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#include "OrderedPly.h"

#define MAX_DATA	9	// max number of input files

struct Params {
  double focal_length;
  double center_u;
  double center_v;
};

static bool is_valid_point(const double x, const double y, const double z)
{
  static const double EPS = 1.0e-12;
  return (fabs(x) > EPS) || (fabs(y) > EPS) || (fabs(z) > EPS);
}

static Params estimate_parameter(const OrderedPly data[MAX_DATA])
{
  double sum_XXYY = 0;
  double sum_X    = 0;
  double sum_Y    = 0;
  double sum_uXvY = 0;
  double sum_u    = 0;
  double sum_v    = 0;
  int    n        = 0;

  for (int i = 0; i < MAX_DATA; i++) {
    if (data[i].size == 0) break;     
    int p = 0;
    for (int v = 0; v < data[i].frame_height; v++) {
      for (int u = 0; u < data[i].frame_width; u++, p++) {
	const double x = data[i].point.at<float>(p, 0);
	const double y = data[i].point.at<float>(p, 1);
	const double z = data[i].point.at<float>(p, 2);
	if (is_valid_point(x, y, z)) {
	  const double X = x / z;
	  const double Y = y / z;
	  sum_XXYY += X * X + Y * Y;
	  sum_X    += X;
	  sum_Y    += Y;
	  sum_uXvY += u * X + v * Y;
	  sum_u    += u;
	  sum_v    += v;
	  n++;
	}
      } // (u)
    } // (v)
  } // (i)

  cv::Mat m = (cv::Mat_<double>(3, 3) << sum_XXYY, sum_X, sum_Y,
	                                 sum_X,        n,     0,
	                                 sum_Y,        0,     n);
  cv::Mat v = (cv::Mat_<double>(3, 1) << sum_uXvY, sum_u, sum_v);
  cv::Mat a = (cv::Mat_<double>(3, 1) << 0, 0, 0);

  if (!cv::solve(m, v, a)) {
    throw std::runtime_error("error: not found solution.");
  }

  Params params = { a.at<double>(0,0), a.at<double>(1,0), a.at<double>(2,0) };
  return params;
}

static void write_parameter(const Params* param)
{
  std::cout << std::setprecision(12);
  std::cout << std::setw(15) << std::right << param->focal_length
	    << std::setw(15) << std::right << "0"
	    << std::setw(15) << std::right << param->center_u
	    << std::endl
	    << std::setw(15) << std::right << "0"
	    << std::setw(15) << std::right << param->focal_length
	    << std::setw(15) << std::right << param->center_v
	    << std::endl
	    << std::setw(15) << std::right << "0"
	    << std::setw(15) << std::right << "0"
	    << std::setw(15) << std::right << "1"
	    << std::endl;
  return;
}

// command line options
const char* keys = {
  "{h|help         |false | print this message.                     }"
  "{1|             |      | input point cloud (PhoXi ply file).     }"
  "{2|             |      | 2nd input point cloud [optional].       }"
  "{3|             |      | 3rd input point cloud [optional].       }"
  "{4|             |      | 4th input point cloud [optional].       }"
  "{5|             |      | 5th input point cloud [optional].       }"
  "{6|             |      | 6th input point cloud [optional].       }"
  "{7|             |      | 7th input point cloud [optional].       }"
  "{8|             |      | 8th input point cloud [optional].       }"
  "{9|             |      | 9th input point cloud [optional].       }"
};

int main(int argc, char** argv)
{
  cv::CommandLineParser parser(argc, argv, keys);

  if (parser.get<bool>("h")) {
    parser.printParams();
    return EXIT_SUCCESS;
  }

  try {
    char number[MAX_DATA][2] = { "1", "2", "3", "4", "5", "6", "7", "8", "9" };
    OrderedPly data[MAX_DATA];

    for (int i = 0; i < MAX_DATA; i++) {
      const std::string input = parser.get<std::string>(number[i]);
      if (input.empty()) break;
      OPlyReader(input, data[i]);
    }
    
    Params params = estimate_parameter(data);
    write_parameter(&params);
  } catch (std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
