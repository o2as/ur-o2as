// *********************************************************************
//              (C) Copyright OMRON CORPORATION 2017
//                        All Rights Reserved
// ---------------------------------------------------------------------
//               ハンドアイキャリブレーション(HANDEYE)ライブラリ
// *********************************************************************
// フ ァ イ ル 名 : handeye_api.h
// 処  理  概  要 : ハンドアイキャリブレーションAPI関数の定義
// 参  考  文  献 : 1) Daniilidis, Konstantinos. "Hand-eye calibration using dual quaternions."
//                      The International Journal of Robotics Research 18.3 (1999): 286-298.
// *********************************************************************

#ifndef MBP_HANDEYE_API_H_
#define MBP_HANDEYE_API_H_

// *************************************
//         インクルードファイル         
// *************************************
#include "mbp_api.h"

// for DLL compile
#ifdef _USRDLL
#if defined( __cplusplus )
#define HANDEYE_API extern "C" __declspec (dllexport)
extern "C" {
#else
#define HANDEYE_API __declspec (dllexport)
#endif

#else
// for STLIB compile
#if defined( __cplusplus )
extern "C" {
#endif
#define HANDEYE_API
#endif

	// ハンドアイキャリブレーションの結果構造体
	typedef struct tagMBPHandEyeResult
	{
		// 固定カメラ：Sen->Rob
		// ハンド搭載カメラ：Sen->Tol
		double rot_mat_sen[9];				// 回転行列
		double handeye_angle_trans_sen[6];	// 結果 0:X軸回転角度,1:Y軸回転角度,2:Z軸回転角度,3:Tx,4:Ty,5:Tz
		double eval_error_sen[6];			// 評価誤差 0:X軸回転角度,1:Y軸回転角度,2:Z軸回転角度,3:Tx,4:Ty,5:Tz

		// 固定カメラ：Obj->Tol
		// ハンド搭載カメラ：Obj->Rob
		double rot_mat_obj[9];				// 回転行列
		double handeye_angle_trans_obj[6];	// 結果 0:X軸回転角度,1:Y軸回転角度,2:Z軸回転角度,3:Tx,4:Ty,5:Tz
		double eval_error_obj[6];			// 評価誤差 0:X軸回転角度,1:Y軸回転角度,2:Z軸回転角度,3:Tx,4:Ty,5:Tz
	}  HANDEYE_HandEyeResult;

	// 固定カメラのパラメータ構造体
	typedef struct tagFixedCamSetting
	{
		// システム環境設定
		float cam_org_in_base[3];	// ベース座標系からセンサ座標系への並進ベクトル [mm]
		float cam_xaxis_in_base[3];	// ベース座標系で見たセンサ座標系X軸方向ベクトル
		float cam_zaxis_in_base[3];	// ベース座標系で見たセンサ座標系Z軸方向ベクトル
		float init_angle_J6;		// J6の初期角度 [deg]
		float targ_org_in_tool[3];	// ツール座標系で見たターゲット原点位置 [mm]

		// 動作設定
		INT32 num_x;				// Xのサンプル数
		INT32 num_y;				// Yのサンプル数
		INT32 num_z;				// Zのサンプル数

		// 動作範囲
		float near_rectangle_center[3];	// センサに一番近い面の動作中心 [mm]
		float operation_range_size[3];	// 動作範囲 [mm]

		float perturb_rot_angle[3];	// センサ座標系の摂動角度 [deg]
	} HANDEYE_FixedCamSetting;

	// ハンド搭載カメラのパラメータ構造体
	typedef struct tagMovingCamSetting
	{
		// システム環境設定
		float trans_targ2base[3];	// ロボットベースから見たターゲットの３次元位置座標 [mm]
		float cam_org_in_tool[3];	// ツール座標系から見たセンサ座標系原点の位置 [mm]
		float cam_xaxis_in_tool[3];	// ツール座標系から見たセンサ座標系のX軸方向
		float cam_zaxis_in_tool[3];	// ツール座標系から見たセンサ座標系のZ軸方向

		// 動作設定
		INT32 num_radius;			// 半径のサンプル数
		INT32 num_theta;			// 極角のサンプル数
		INT32 num_phi;				// 方位角のサンプル数
		float *pos_radius;			// 半径位置(num_radius個) [mm]
		float *pos_theta;			// 極角(num_theta個) [deg]
		float *pos_phi;				// 方位角(num_phi個) [deg]

		// 摂動量（ターゲット座標系でのセンサ位置姿勢の摂動量）
		INT32 delta_rot_x;			// 角度x [deg]
		INT32 delta_rot_y;			// 角度y [deg]
		INT32 delta_rot_z;			// 角度z [deg]
		INT32 delta_pos_x;			// 位置x [mm]
		INT32 delta_pos_y;			// 位置y [mm]
		INT32 delta_pos_z;			// 位置z [mm]
	} HANDEYE_MovingCamSetting;

	// *************************************
	//         プロトタイプ関数宣言        
	// *************************************
	// 主要関数
	HANDEYE_API MBP_ERR_CODE HANDEYE_CalibrateFixedCam(const INT32 sample_num, const double *robot_pose_angle, const double *robot_pose_trans, const double *target_pose_angle, const double *target_pose_trans, HANDEYE_HandEyeResult *handeye_result);
	HANDEYE_API MBP_ERR_CODE HANDEYE_CalibrateFixedCamWithKnownTcp(const INT32 sample_num, const double *robot_pose_angle, const double *robot_pose_trans, const double *target_pose_angle, const double *target_pose_trans, HANDEYE_HandEyeResult *handeye_result);
	HANDEYE_API MBP_ERR_CODE HANDEYE_CalibrateMovingCam(const INT32 sample_num, const double *robot_pose_angle, const double *robot_pose_trans, const double *target_pose_angle, const double *target_pose_trans, HANDEYE_HandEyeResult *handeye_result);
	HANDEYE_API MBP_ERR_CODE HANDEYE_Evaluate(const INT32 sample_num, const double *robot_pose_angle, const double *robot_pose_trans, const double *target_pose_angle, const double *target_pose_trans, const HANDEYE_HandEyeResult *handeye_param_compared, const BOOL is_fixed_cam, double eval_error[6], double *predictedpose_rot_mat, double *predictedpose_trans_vec);
	HANDEYE_API MBP_ERR_CODE HANDEYE_MakeRobotPoseFixedCam(const HANDEYE_FixedCamSetting *setting, float *angle_tool2base, float *trans_tool2base);	// 固定カメラで位置姿勢生成
	HANDEYE_API MBP_ERR_CODE HANDEYE_MakeRobotPoseMovingCam(const HANDEYE_MovingCamSetting *setting, float *angle_tool2base, float *trans_tool2base);	// ハンド搭載カメラで位置姿勢生成

	// メモリ、初期化関連
	HANDEYE_API MBP_ERR_CODE HANDEYE_AllocHandEyeResult(HANDEYE_HandEyeResult **handeye_result);
	HANDEYE_API MBP_ERR_CODE HANDEYE_DeleteHandEyeResult(HANDEYE_HandEyeResult **handeye_result);
	HANDEYE_API MBP_ERR_CODE HANDEYE_AllocFixedCamSetting(HANDEYE_FixedCamSetting **setting);
	HANDEYE_API MBP_ERR_CODE HANDEYE_DeleteFixedCamSetting(HANDEYE_FixedCamSetting **setting);
	HANDEYE_API MBP_ERR_CODE HANDEYE_AllocMovingCamSetting(HANDEYE_MovingCamSetting **setting);
	HANDEYE_API MBP_ERR_CODE HANDEYE_DeleteMovingCamSetting(HANDEYE_MovingCamSetting **setting);

	// サブ処理
	HANDEYE_API MBP_ERR_CODE HANDEYE_GetVersion(char *major, char *minor, char *sub_minor);	// ライブラリーバージョンの取得
	HANDEYE_API char*        HANDEYE_GetErrorMessage(const INT32 error_code);

#if defined( __cplusplus )
}
#endif

#endif	/* MBP_HANDEYE_API_H_ */
