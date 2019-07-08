/*************************************************************************************/
/*
/*  MBP (MOGRA for Bin Picking)
/*  3D CAD Matching: 
/*  6D object pose estimation based on view-based template approach
/*
/*  Copyright (C) 2016, 2017  OMRON Corporation All rights reserved.
/*
/**************************************************************************************/

#ifndef MBP_CAD_API_H_
#define MBP_CAD_API_H_

//#include <windows.h>
#include "mbp_api.h"

// for DLL compile
#ifdef _USRDLL
#if defined( __cplusplus )
#define CAD_API extern "C" __declspec (dllexport)
extern "C" {
#else
#define CAD_API __declspec (dllexport)
#endif

// for STLIB compile
#else
#if defined( __cplusplus )
extern "C" {
#endif
#define CAD_API
#endif
	
typedef void CAD_ModelData;
typedef void CAD_TrainParam;
typedef void CAD_SearchInfo;
	
	// Training
	CAD_API MBP_ERR_CODE CAD_AllocTrainParam(CAD_TrainParam **train_param);
	CAD_API MBP_ERR_CODE CAD_DeleteTrainParam(CAD_TrainParam **train_param);	
	CAD_API MBP_ERR_CODE CAD_SetTrainParam(const MBP_Setting *mbp_setting, const INT32 object_id, const BOOL both_sided, const INT32 min_lat, const INT32 max_lat, const INT32 min_lon, const INT32 max_lon, const INT32 min_dist, const INT32 max_dist, const float ini_pose[3], const INT32 thread_num, CAD_TrainParam *cad_train_param);
	CAD_API MBP_ERR_CODE CAD_Train(const CAD_TrainParam *cad_train_param, MBP_Mesh *mbp_mesh_obj, MBP_Mesh *mbp_mesh_train, MBP_Meshes *mbp_mesh_weight, CAD_ModelData *cad_model_data, INT32 *model_id);
	
	// Model data
	CAD_API MBP_ERR_CODE CAD_AllocModelData(CAD_ModelData  **model_data);
	CAD_API MBP_ERR_CODE CAD_DeleteAllModelData(CAD_ModelData  **model_data);
	CAD_API MBP_ERR_CODE CAD_DeleteModelData(const INT32 model_id, CAD_ModelData *model_data);
	CAD_API MBP_ERR_CODE CAD_ValidateModelData(const INT32 model_id, CAD_ModelData *model_data);
	CAD_API MBP_ERR_CODE CAD_InvalidateModelData(const INT32 model_id, CAD_ModelData *model_data);
	CAD_API MBP_ERR_CODE CAD_SaveModelData(const char *filename, const CAD_ModelData *model_data);
	CAD_API MBP_ERR_CODE CAD_LoadModelData(const char *filename, CAD_ModelData *model_data);
	CAD_API MBP_ERR_CODE CAD_GetModelDataSize(const CAD_ModelData *cad_model_data, INT32 *model_data_size);
	CAD_API MBP_ERR_CODE CAD_CopyModelData(const CAD_ModelData *cad_model_data, const INT32 model_data_size, UINT8 *pack_model_data);
	CAD_API MBP_ERR_CODE CAD_SaveModelDataTxt(const char *model_filename, const double time_train, const CAD_ModelData *cad_model_data);
	
	// Search
	CAD_API MBP_ERR_CODE CAD_AllocSearchInfo(CAD_SearchInfo **search_info);
	CAD_API MBP_ERR_CODE CAD_DeleteSearchInfo(CAD_SearchInfo **search_info);
	CAD_API MBP_ERR_CODE CAD_SetGlobalSearchParam(const INT32 width, const INT32 height, const INT32 search_color_num, const INT32 icp_color_num, const float thresh_depth, const INT32 thresh_grad, const INT32 thresh_inlier_pc, const RECT search_area, const INT32 max_result_num_all, const INT32 thread_num, CAD_SearchInfo *cad_search_info);
	CAD_API MBP_ERR_CODE CAD_SetObjectSearchParam(const INT32 model_id, const INT32 min_lat, const INT32 max_lat, const INT32 min_lon, const INT32 max_lon, const INT32 min_roll, const INT32 max_roll, const INT32 min_dist, const INT32 max_dist, const INT32 thresh_search, const float search_coef, const INT32 max_result_num, CAD_SearchInfo *cad_search_info);
	CAD_API MBP_ERR_CODE CAD_PrepareForSearch(CAD_ModelData *model_data, CAD_SearchInfo *search_info);	
	CAD_API MBP_ERR_CODE CAD_GetSearchAreaFromMaskImage(const UINT8 *mask_image, const INT32 width, const INT32 height, RECT *search_area);
	CAD_API MBP_ERR_CODE CAD_Search(const float *src_pc, const UINT8 *src_image, const UINT8 *mask_image, const CAD_ModelData *cad_model_data, CAD_SearchInfo *cad_search_info, MBP_SearchResult *search_result);
	
	// Other
	CAD_API MBP_ERR_CODE CAD_DrawResultImage(const MBP_SearchResult *search_result, const CAD_ModelData *cad_model_data, const INT32 width, const INT32 height, const BOOL flag_drawfast, UINT8 *edge_image, float *point_cloud);
	CAD_API MBP_ERR_CODE CAD_MakeGradImage(const CAD_ModelData *model_data, const CAD_SearchInfo *search_info, const INT32 model_id, INT32 *width, INT32 *height, UINT8 **grad_img);
	CAD_API MBP_ERR_CODE CAD_MakeNormImage(const CAD_ModelData *model_data, const CAD_SearchInfo *search_info, const INT32 model_id, INT32 *width, INT32 *height, UINT8 **norm_img);
	CAD_API char* CAD_GetErrorMessage(const INT32 error_code);
	
#if defined( __cplusplus )
}
#endif

#endif	/* MBP_CAD_API_H_ */
