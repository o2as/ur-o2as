//
// OrderedPly.cpp
//
#include <iostream>
#include <exception>
#include <cstdlib>
#include <cassert>
#include <vector>
#include "OrderedPly.h"
#include "rply.h"

enum VertexProperty {
  X, Y, Z, NX, NY, NZ, RED, GREEN, BLUE, TEXTURE32, DEPTH32
};

enum CameraProperty {
  VIEW_PX, VIEW_PY, VIEW_PZ, X_AXISX, X_AXISY, X_AXISZ,
  Y_AXISX, Y_AXISY, Y_AXISZ, Z_AXISX, Z_AXISY, Z_AXISZ
};

enum PhoXiFrameParamsProperty {
  FRAME_WIDTH, FRAME_HEIGHT, FRAME_INDEX, FRAME_START_TIME, FRAME_DURATION
};

extern "C" {
  //
  // callback function to read 3D points
  //
  int read_vertex(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// index of property

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= X) && (n <= DEPTH32));

    value = ply_get_argument_value(argument);	// property

    if ((n == X) || (n == Y) || (n == Z)) {
      data->point.at<float>(data->size, n) = value;
    } else if ((n == NX) || (n == NY) || (n == NZ)) {
      data->normal.at<float>(data->size, n - NX) = value;
    } else if ((n == RED) || (n == GREEN) || (n == BLUE)) {
      data->color.at<uchar>(data->size, n - RED) = value;
    } else if (n == TEXTURE32) {
      data->texture[data->size] = value;
    } else {
      assert(n == DEPTH32);
      data->depth[data->size] = value;
    }

    if (n == data->last) {
      data->size++;
    }
    return 1;
  }


  //
  // callback function to read camera properties
  //
  int read_camera(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// index of property

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= VIEW_PX) && (n <= Z_AXISZ));

    value = ply_get_argument_value(argument);	// property

    if (n == VIEW_PX) {
      data->view[0] = value;
    } else if (n == VIEW_PY) {
      data->view[1] = value;
    } else if (n == VIEW_PZ) {
      data->view[2] = value;
    } else if (n == X_AXISX) {
      data->x_axis[0] = value;
    } else if (n == X_AXISY) {
      data->x_axis[1] = value;
    } else if (n == X_AXISZ) {
      data->x_axis[2] = value;
    } else if (n == Y_AXISX) {
      data->y_axis[0] = value;
    } else if (n == Y_AXISY) {
      data->y_axis[1] = value;
    } else if (n == Y_AXISZ) {
      data->y_axis[2] = value;
    } else if (n == Z_AXISX) {
      data->z_axis[0] = value;
    } else if (n == Z_AXISY) {
      data->z_axis[1] = value;
    } else {
      assert(n == Z_AXISZ);
      data->z_axis[2] = value;
    }

    return 1;
  }

  //
  // callback function to read frame properties
  //
  int read_phoxi_frame_params(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= FRAME_WIDTH) && (n <= FRAME_DURATION));

    value = ply_get_argument_value(argument);

    if (n == FRAME_WIDTH)
    {
      data->frame_width = value;
    } else if (n == FRAME_HEIGHT) {
      data->frame_height = value;
    } else if (n == FRAME_INDEX) {
      data->frame_index = value;
    } else if (n == FRAME_START_TIME) {
      data->frame_start_time = value;
    } else {
      assert(n == FRAME_DURATION);
      data->frame_duration = value;
    }

    return 1;
  }

} // extern "C"

OPlyReader::OPlyReader(std::string inputFile, OrderedPly& inputData):
  filename(inputFile), data(inputData)
{
  p_ply src = ply_open(filename.c_str(), NULL, 0, NULL);
  if (!src) {
    throw std::runtime_error("error: failed to open input file.");
  }

  if (!ply_read_header(src)) {
    ply_close(src);
    throw std::runtime_error("error: failed to read file header.");
  }

  p_ply_element element = NULL;

  while ((element = ply_get_next_element(src, element))) {
    const char* element_name = NULL;
    long ninstances = 0;
    ply_get_element_info(element, &element_name, &ninstances);

    std::string en = element_name;

    if (en == "vertex") {
      data.point   = cv::Mat_<float>(ninstances, 3);
      data.normal  = cv::Mat_<float>(ninstances, 3);
      data.color   = cv::Mat_<uchar>(ninstances, 3);
      data.texture = std::vector<float>(ninstances);
      data.depth   = std::vector<float>(ninstances);
      data.size    = 0;
      data.last    = X;
    }

    p_ply_property property = NULL;
    while ((property = ply_get_next_property(element, property))) {

      const char* property_name = NULL;
      ply_get_property_info(property, &property_name, NULL, NULL, NULL);

      std::string pn = property_name;
      int ok;

      if ((en == "vertex") && (pn == "x")) {
        ok = ply_set_read_cb(src, "vertex", "x", read_vertex, &data, X);
	data.last = X;
      } else if ((en == "vertex") && (pn == "y")) {
        ok = ply_set_read_cb(src, "vertex", "y", read_vertex, &data, Y);
	data.last = Y;
      } else if ((en == "vertex") && (pn == "z")) {
        ok = ply_set_read_cb(src, "vertex", "z", read_vertex, &data, Z);
	data.last = Z;
      } else if ((en == "vertex") && (pn == "nx")) {
        ok = ply_set_read_cb(src, "vertex", "nx", read_vertex, &data, NX);
	data.last = NX;
      } else if ((en == "vertex") && (pn == "ny")) {
        ok = ply_set_read_cb(src, "vertex", "ny", read_vertex, &data, NY);
	data.last = NY;
      } else if ((en == "vertex") && (pn == "nz")) {
        ok = ply_set_read_cb(src, "vertex", "nz", read_vertex, &data, NZ);
	data.last = NZ;
      } else if ((en == "vertex") && (pn == "red")) {
        ok = ply_set_read_cb(src, "vertex", "red", read_vertex, &data, RED);
	data.last = RED;
      } else if ((en == "vertex") && (pn == "green")) {
        ok = ply_set_read_cb(src, "vertex", "green", read_vertex, &data, GREEN);
	data.last = GREEN;
      } else if ((en == "vertex") && (pn == "blue")) {
        ok = ply_set_read_cb(src, "vertex", "blue", read_vertex, &data, BLUE);
	data.last = BLUE;
      } else if ((en == "vertex") && (pn == "Texture32")) {
        ok = ply_set_read_cb(src, "vertex", "Texture32",
			     read_vertex, &data, TEXTURE32);
	data.last = TEXTURE32;
      } else if ((en == "vertex") && (pn == "Depth32")) {
        ok = ply_set_read_cb(src, "vertex", "Depth32",
			     read_vertex, &data, DEPTH32);
	data.last = DEPTH32;
      } else if ((en == "camera") && (pn == "view_px")) {
        ok = ply_set_read_cb(src, "camera", "view_px",
			     read_camera, &data, VIEW_PX);
      } else if ((en == "camera") && (pn == "view_py")) {
        ok = ply_set_read_cb(src, "camera", "view_py",
			     read_camera, &data, VIEW_PY);
      } else if ((en == "camera") && (pn == "view_pz")) {
        ok = ply_set_read_cb(src, "camera", "view_pz",
			     read_camera, &data, VIEW_PZ);
      } else if ((en == "camera") && (pn == "x_axisx")) {
        ok = ply_set_read_cb(src, "camera", "x_axisx",
			     read_camera, &data, X_AXISX);
      } else if ((en == "camera") && (pn == "x_axisy")) {
        ok = ply_set_read_cb(src, "camera", "x_axisy",
			     read_camera, &data, X_AXISY);
      } else if ((en == "camera") && (pn == "x_axisz")) {
        ok = ply_set_read_cb(src, "camera", "x_axisz",
			     read_camera, &data, X_AXISZ);
      } else if ((en == "camera") && (pn == "y_axisx")) {
        ok = ply_set_read_cb(src, "camera", "y_axisx",
			     read_camera, &data, Y_AXISX);
      } else if ((en == "camera") && (pn == "y_axisy")) {
        ok = ply_set_read_cb(src, "camera", "y_axisy",
			     read_camera, &data, Y_AXISY);
      } else if ((en == "camera") && (pn == "y_axisz")) {
        ok = ply_set_read_cb(src, "camera", "y_axisz",
			     read_camera, &data, Y_AXISZ);
      } else if ((en == "camera") && (pn == "z_axisx")) {
        ok = ply_set_read_cb(src, "camera", "z_axisx",
			     read_camera, &data, Z_AXISX);
      } else if ((en == "camera") && (pn == "z_axisy")) {
        ok = ply_set_read_cb(src, "camera", "z_axisy",
			     read_camera, &data, Z_AXISY);
      } else if ((en == "camera") && (pn == "z_axisz")) {
        ok = ply_set_read_cb(src, "camera", "z_axisz",
			     read_camera, &data, Z_AXISZ);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_width")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_width",
			     read_phoxi_frame_params, &data, FRAME_WIDTH);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_height")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_height",
			     read_phoxi_frame_params, &data, FRAME_HEIGHT);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_index")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_index",
			     read_phoxi_frame_params, &data, FRAME_INDEX);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_start_time")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_start_time",
			     read_phoxi_frame_params, &data, FRAME_START_TIME);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_duration")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_duration",
			     read_phoxi_frame_params, &data, FRAME_DURATION);
      } else {
	// ignore unknown elements and properties. 
	ok = 1;
      }

      if (!ok) {
	ply_close(src);
	throw std::runtime_error("error: failed to read point cloud.");
      }
    } // while (property)
  } // while (element)

  if (!ply_read(src)) {
    ply_close(src);
    throw std::runtime_error("error: failed to read data body.");
  }

  if (!ply_close(src)) {
    throw std::runtime_error("error: failed to close input file.");
  }

  if (data.size != data.frame_width * data.frame_height) {
    throw std::runtime_error("error: size mismatch. invalid ordered ply.");
  }

  return;
}
