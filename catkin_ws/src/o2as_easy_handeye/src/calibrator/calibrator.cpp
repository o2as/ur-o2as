/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Calibrator node
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/

#include "calibrator.h"
#include <visp_bridge/3dpose.h>
#include "names.h"
#include "aistCalibration.h"
#include <visp/vpHomogeneousMatrix.h>

#define USE_AIST_CALIBRATION
#define DEBUG

namespace visp_hand2eye_calibration
{
Calibrator::Calibrator()
    :n_(),
     compute_effector_camera_service_(
	 n_.advertiseService(compute_effector_camera_service,
			     &Calibrator::computeEffectorCameraCallback,
			     this)),
     compute_effector_camera_quick_service_(
	 n_.advertiseService(compute_effector_camera_quick_service,
			     &Calibrator::computeEffectorCameraQuickCallback,
			     this)),
     reset_service_(n_.advertiseService(reset_service,
					&Calibrator::resetCallback, this)),
     check_inputs_(ros::NodeHandle(), ros::this_node::getName()),
     queue_size_(1000)
{
    ROS_INFO("o2as_easy_handeye_calibrator: initializing calibrator...");

    ros::V_string	topics;
    topics.push_back(camera_object_topic);
    topics.push_back(world_effector_topic);
    check_inputs_.start(topics, 60.0);
    if (!ros::ok())
      return;

  //define subscribers
    camera_object_subscriber_
	= n_.subscribe(camera_object_topic, queue_size_,
		       &Calibrator::cameraObjectCallback, this);
    world_effector_subscriber_
	= n_.subscribe(world_effector_topic, queue_size_,
		       &Calibrator::worldEffectorCallback, this);
}

Calibrator::~Calibrator()
{
}
    
void
Calibrator::spin()
{
    ros::spin();
}

void
Calibrator::cameraObjectCallback(
    const geometry_msgs::Transform::ConstPtr& trans)
{
    ROS_DEBUG("o2as_easy_handeye_calibrator: new cMo: [%f,%f,%f -- %f,%f,%f,%f]",
	      trans->translation.x, trans->translation.y, trans->translation.z,
	      trans->rotation.x, trans->rotation.y, trans->rotation.z,
	      trans->rotation.w);

    const auto	cMo = visp_bridge::toVispHomogeneousMatrix(*trans);
    cMo_vec_.push_back(cMo);
}

void
Calibrator::worldEffectorCallback(
    const geometry_msgs::Transform::ConstPtr& trans)
{
    ROS_DEBUG("o2as_easy_handeye_calibrator: new wMe: [%f,%f,%f -- %f,%f,%f,%f]",
	      trans->translation.x, trans->translation.y, trans->translation.z,
	      trans->rotation.x, trans->rotation.y, trans->rotation.z,
	      trans->rotation.w);

    const auto	wMe = visp_bridge::toVispHomogeneousMatrix(*trans);
    wMe_vec_.push_back(wMe);
}

bool
Calibrator::computeEffectorCameraCallback(
    compute_effector_camera::Request&  req,
    compute_effector_camera::Response& res)
{
    if (cMo_vec_.size() != wMe_vec_.size() || wMe_vec_.size() < 2)
    {
      ROS_ERROR("o2as_easy_handeye_calibrator: transformation vectors have different sizes or contain too few elements");
      return false;
    }

    ROS_INFO("o2as_easy_handeye_calibrator: computing %d values...",
	     (int)wMe_vec_.size());

    vpHomogeneousMatrix	eMc;
    aistCalibration::calibrationTsai(cMo_vec_, wMe_vec_, eMc);

    res.effector_camera = visp_bridge::toGeometryMsgsTransform(eMc);

    return true;
}

bool
Calibrator::computeEffectorCameraQuickCallback(
    compute_effector_camera_quick::Request&  req,
    compute_effector_camera_quick::Response& res)
{
    TransformArray	camera_object  = req.camera_object;
    TransformArray	world_effector = req.world_effector;

    if (camera_object.transforms.size() != world_effector.transforms.size() ||
	world_effector.transforms.size() < 2)
    {
	ROS_ERROR("o2as_easy_handeye_calibrator: transformation vectors have different sizes or contain too few elements");
	return false;
    }

    ROS_INFO("o2as_easy_handeye_calibrator: computing...");

#ifdef USE_AIST_CALIBRATION
    std::vector<aistCalibration::Transform>	cMo;
    std::vector<aistCalibration::Transform>	wMe;
    for (unsigned int i = 0; i < camera_object.transforms.size(); i++)
    {
	cMo.push_back(aistCalibration::Transform(
			  camera_object.transforms[i]));
	wMe.push_back(aistCalibration::Transform(
			  world_effector.transforms[i]));
    }

    res.effector_camera = aistCalibration::calibrationAIST(cMo, wMe);
#else
    std::vector<vpHomogeneousMatrix>	cMo_vp;
    std::vector<vpHomogeneousMatrix>	wMe_vp;
    for (unsigned int i = 0; i < camera_object.transforms.size(); i++)
    {
	cMo_vp.push_back(visp_bridge::toVispHomogeneousMatrix(
			      camera_object.transforms[i]));
	wMe_vp.push_back(visp_bridge::toVispHomogeneousMatrix(
			      world_effector.transforms[i]));
    }

    vpHomogeneousMatrix		eMc_vp;
    aistCalibration::calibrationTsai(cMo_vp, wMe_vp, eMc_vp);

    res.effector_camera = visp_bridge::toGeometryMsgsTransform(eMc_vp);
#endif

#ifdef DEBUG
#  ifndef USE_AIST_CALIBRATION
    std::vector<aistCalibration::Transform>	cMo;
    std::vector<aistCalibration::Transform>	wMe;
    for (unsigned int i = 0; i < camera_object.transforms.size(); i++)
    {
	cMo.push_back(aistCalibration::Transform(
			  camera_object.transforms[i]));
	wMe.push_back(aistCalibration::Transform(
			  world_effector.transforms[i]));
    }
#  endif
    aistCalibration::Transform		eMc(res.effector_camera);
    std::ofstream			out("cMo_wMe_pairs.txt");
    out << cMo.size() << std::endl;
    for (size_t n = 0; n < cMo.size(); ++n)
	out << cMo[n] << std::endl
	    << wMe[n] << std::endl << std::endl;

    const auto	wMo = objectToWorld(cMo, wMe, eMc);
    evaluateAccuracy(out, cMo, wMe, eMc, wMo);
#endif
    return true;
}

bool
Calibrator::resetCallback(reset::Request& req, reset::Response& res)
{
    ROS_INFO("o2as_easy_handeye_calibrator: reseting...");
    cMo_vec_.clear();
    wMe_vec_.clear();

    return true;
}

}
