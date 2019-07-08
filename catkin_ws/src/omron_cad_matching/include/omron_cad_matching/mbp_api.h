/*************************************************************************************/
/*
/*  MBP (MOGRA for Bin Picking)
/*  3D CAD Matching: 
/*  6D object pose estimation based on view-based template approach
/*
/*  Copyright (C) 2016, 2017  OMRON Corporation All rights reserved.
/*
/**************************************************************************************/

#ifndef MBP_API_H_
#define MBP_API_H_

// *************************************
//         インクルードファイル
// *************************************
//#include <windows.h>
#include "mbp_error.h"

// for DLL compile
#ifdef _USRDLL
#if defined( __cplusplus )
#define MBP_API extern "C" __declspec (dllexport)
extern "C" {
#else
#define MBP_API __declspec (dllexport)
#endif

// for STLIB compile
#else
#if defined( __cplusplus )
extern "C" {
#endif
#define MBP_API
#endif


// *************************************
//      　　　　型定義
// *************************************
typedef void MBP_Mesh;
typedef void MBP_Meshes;
typedef void MBP_Box;

// *************************************
//     　　　　列挙体定義
// *************************************
// 障害物の種類
typedef enum tagMbpObstacle {
	MBP_OBSTACLE_BOX = 0,
	MBP_OBSTACLE_FLOOR = 1,
	MBP_OBSTACLE_NOTHING = 2
} e_MBP_Obstacle;

// 回転角度の回転順
typedef enum tagMbpRotAngleOrder
{
	MBP_ROTANGLE_XYZ = 0,
	MBP_ROTANGLE_ZYZ = 1,
} e_MBP_RotAngleOrder;

// *************************************
//      　　　　構造体定義
// *************************************
typedef struct tagMBPSetting
{
	INT32				width;					// 画像幅 [pix]
	INT32				height;					// 画像高さ [pix]
	float				cam_param[9];			// カメラ内部パラメータ
	float				dist_param[5];			// 歪みパラメータ
	e_MBP_Obstacle		obstacle;				// 障害物の種類　MBP_OBSTACLE_BOX:箱、MBP_OBSTACLE_FLOOR:床、MBP_OBSTACLE_NOTHING:障害物無し
	float				floor_plane[4];	        // 床面を表す平面の式の係数
												// aX + bY + cZ + d = 0 の(a,b,c,d)
												// 例えば、カメラに正対な500mmの距離にある床面は(a,b,c,d) = (0,0,-1,500)
	float               max_height_from_floor;  // 床面からバラ積みの物体の頂点までの高さ [mm]
	float				box_width;				// コンテナの幅  (カメラ座標系でX軸) [mm]
	float				box_depth;				// コンテナの奥行(カメラ座標系でY軸) [mm]
	float				box_height;				// コンテナの高さ(カメラ座標系でZ軸) [mm]
	float				box_thickness;			// コンテナの淵の厚み [mm]
	float				box_bottom_height;		// コンテナの底面の厚み [mm]
	float				handeye_rot_angle[3];	// ハンドアイキャリブレーション結果の回転角度 [deg]
	float				handeye_trans[3];		// ハンドアイキャリブレーション結果の並進ベクトル [mm]
	float				handeye_rot_mat[9];		// ハンドアイキャリブレーション結果の回転行列
	BOOL				is_fixed_cam;           // 固定カメラかを指定するフラグ(TRUE: 固定カメラ、FALSE: ハンド搭載カメラ)
} MBP_Setting;

typedef struct tagMBPSearchResult
{
	INT32				result_num;				// 検出結果個数
	INT32				*object_id;				// 物体のID(ユーザーが登録時に設定します)
	UINT8				*model_id;				// モデルのID(APIが登録時に割り当てます)
	float				*score2D;				// 物体の輪郭形状がモデルとどの程度一致するかを表すスコア
	float				*score3D;				// 物体の表面形状がモデルとどの程度一致するかを表すスコア
	float				*score_final;			// 2Dと3Dのスコアを統合した最終スコア
	BOOL				*valid_flag;			// 基本的にTRUEが格納されます。結果描画時にFALSEを与えると描画されません。
	float				**pos3D;				// センサ座標系におけるオブジェクト座標原点の3次元位置 [mm]
	float				**rot3D;				// センサ座標系におけるオブジェクト座標系の回転角度
	float				**rot_mat;				// センサ座標系におけるオブジェクト座標系の回転行列
	float				**obj_cog;				// センサ座標系におけるオブジェクト重心位置座標の3次元位置 [mm]
	INT32				**axis;					// センサ座標系におけるオブジェクト座標系の各軸を2次元画像に投影した座標
} MBP_SearchResult;

typedef struct tagMBPGraspResult
{
	// 認識した把持点数
	INT32				result_num;

	// pick_***_tool2cam?ｿｽﾍ??ｿｽ?ｿｽ{?ｿｽb?ｿｽg?ｿｽ?ｿｽ?ｿｽ?ｿｽﾉは用?ｿｽ?ｿｽ?ｿｽﾜゑｿｽ?ｿｽ?ｿｽB?ｿｽm?ｿｽF?ｿｽp?ｿｽﾌ出?ｿｽﾍでゑｿｽ?ｿｽB
	// ?ｿｽJ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉゑｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽs?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽﾅのツ?ｿｽ[?ｿｽ?ｿｽ?ｿｽﾌ位置?ｿｽp?ｿｽ?ｿｽ?ｿｽﾆ難ｿｽ?ｿｽ?ｿｽ?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのピ?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽ?ｿｽJ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの包ｿｽ?ｿｽi?ｿｽx?ｿｽN?ｿｽg?ｿｽ?ｿｽ
	float				**pick_pos3D_tool2cam;//[mm]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのピ?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽJ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽp?ｿｽx
	float				**pick_rot3D_tool2cam;//[deg]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのピ?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽJ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽs?ｿｽ?ｿｽ
	float				**pick_rot_mat_tool2cam;

	// pick_***_tool2base?ｿｽﾍ??ｿｽ?ｿｽ{?ｿｽb?ｿｽg?ｿｽ?ｿｽ?ｿｽ?ｿｽﾉは用?ｿｽ?ｿｽ?ｿｽ?ｿｽl?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉゑｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽs?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽﾅのツ?ｿｽ[?ｿｽ?ｿｽ?ｿｽﾌ位置?ｿｽp?ｿｽ?ｿｽ?ｿｽﾆ難ｿｽ?ｿｽ?ｿｽ?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのピ?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの包ｿｽ?ｿｽi?ｿｽx?ｿｽN?ｿｽg?ｿｽ?ｿｽ
	float				**pick_pos3D_tool2base;//[mm]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのピ?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽp?ｿｽx
	float				**pick_rot3D_tool2base;//[deg]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのピ?ｿｽb?ｿｽL?ｿｽ?ｿｽ?ｿｽO?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽs?ｿｽ?ｿｽ
	float				**pick_rot_mat_tool2base;

	// approach_***_tool2base?ｿｽﾍ??ｿｽ?ｿｽ{?ｿｽb?ｿｽg?ｿｽ?ｿｽ?ｿｽ?ｿｽﾉは用?ｿｽ?ｿｽ?ｿｽ?ｿｽl?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉゑｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽA?ｿｽv?ｿｽ?ｿｽ?ｿｽ[?ｿｽ`?ｿｽﾊ置?ｿｽﾅのツ?ｿｽ[?ｿｽ?ｿｽ?ｿｽﾌ位置?ｿｽp?ｿｽ?ｿｽ?ｿｽﾆ難ｿｽ?ｿｽ?ｿｽ?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのア?ｿｽv?ｿｽ?ｿｽ?ｿｽ[?ｿｽ`?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの包ｿｽ?ｿｽi?ｿｽx?ｿｽN?ｿｽg?ｿｽ?ｿｽ
	float				**approach_pos3D_tool2base;//[mm]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのア?ｿｽv?ｿｽ?ｿｽ?ｿｽ[?ｿｽ`?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽp?ｿｽx
	float				**approach_rot3D_tool2base;//[deg]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅのア?ｿｽv?ｿｽ?ｿｽ?ｿｽ[?ｿｽ`?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽs?ｿｽ?ｿｽ
	float				**approach_rot_mat_tool2base;

	// pullout_***_tool2base?ｿｽﾍ??ｿｽ?ｿｽ{?ｿｽb?ｿｽg?ｿｽ?ｿｽ?ｿｽ?ｿｽﾉは用?ｿｽ?ｿｽ?ｿｽ?ｿｽl?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉゑｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾊ置?ｿｽﾅのツ?ｿｽ[?ｿｽ?ｿｽ?ｿｽﾌ位置?ｿｽp?ｿｽ?ｿｽ?ｿｽﾆ難ｿｽ?ｿｽ?ｿｽ?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅの茨ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの包ｿｽ?ｿｽi?ｿｽx?ｿｽN?ｿｽg?ｿｽ?ｿｽ
	float				**pullout_pos3D_tool2base;//[mm]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅの茨ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽp?ｿｽx
	float				**pullout_rot3D_tool2base;//[deg]
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅの茨ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾊ置?ｿｽ?ｿｽx?ｿｽ[?ｿｽX?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽs?ｿｽ?ｿｽ
	float				**pullout_rot_mat_tool2base;

	// object2tool?ｿｽﾍ??ｿｽ?ｿｽ{?ｿｽb?ｿｽg?ｿｽ?ｿｽ?ｿｽ?ｿｽﾉは用?ｿｽ?ｿｽ?ｿｽﾜゑｿｽ?ｿｽ?ｿｽB?ｿｽV?ｿｽX?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ`?ｿｽ?ｿｽﾉ用?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの出?ｿｽﾍでゑｿｽ?ｿｽB
	// ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾅの包ｿｽ?ｿｽﾌの位置?ｿｽp?ｿｽ?ｿｽ?ｿｽﾆ難ｿｽ?ｿｽ?ｿｽ?ｿｽﾅゑｿｽ?ｿｽB
	// ?ｿｽ?ｿｽ?ｿｽﾌ搾ｿｽ?ｿｽW?ｿｽn?ｿｽ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの包ｿｽ?ｿｽi?ｿｽx?ｿｽN?ｿｽg?ｿｽ?ｿｽ
	float				**pos3D_object2tool;//[mm]
	// ?ｿｽ?ｿｽ?ｿｽﾌ搾ｿｽ?ｿｽW?ｿｽn?ｿｽ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽp?ｿｽx
	float				**rot3D_object2tool;//[deg]
	// ?ｿｽ?ｿｽ?ｿｽﾌ搾ｿｽ?ｿｽW?ｿｽn?ｿｽ?ｿｽc?ｿｽ[?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽW?ｿｽn?ｿｽﾉ変奇ｿｽ?ｿｽ?ｿｽ?ｿｽ驍ｽ?ｿｽﾟの会ｿｽ]?ｿｽs?ｿｽ?ｿｽ
	float				**rot_mat_object2tool;

	// ?ｿｽ?ｿｽ?ｿｽﾌ位置?ｿｽp?ｿｽ?ｿｽ?ｿｽF?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾊのイ?ｿｽ?ｿｽ?ｿｽf?ｿｽb?ｿｽN?ｿｽX
	INT32				*cad_result_id;
	// ?ｿｽc?ｿｽ?ｿｽ?ｿｽﾉ使?ｿｽp?ｿｽ?ｿｽ?ｿｽ?ｿｽn?ｿｽ?ｿｽ?ｿｽh?ｿｽﾌイ?ｿｽ?ｿｽ?ｿｽf?ｿｽb?ｿｽN?ｿｽX
	INT32				*hand_id;
	// ?ｿｽ?ｿｽw?ｿｽn?ｿｽ?ｿｽ?ｿｽh?ｿｽ?ｿｽ?ｿｽz?ｿｽ?ｿｽ?ｿｽn?ｿｽ?ｿｽ?ｿｽh?ｿｽ?ｿｽ?ｿｽ?ｿｽw?ｿｽ閧ｷ?ｿｽ?ｿｽt?ｿｽ?ｿｽ?ｿｽO
	BOOL				*is_twofinger;

	// ■■■■■ 二指のみ ■■■■■
	// Step?ｿｽt?ｿｽ@?ｿｽC?ｿｽ?ｿｽ?ｿｽﾌイ?ｿｽ?ｿｽ?ｿｽf?ｿｽb?ｿｽN?ｿｽX
	INT32				*step_id;
	// ?ｿｽc?ｿｽ?ｿｽ?ｿｽJ?ｿｽn?ｿｽ?ｿｽ?ｿｽﾌハ?ｿｽ?ｿｽ?ｿｽh?ｿｽﾌ開?ｿｽ?ｿｽ?ｿｽ?ｿｽ
	INT32				*stroke_start;//[mm]
	// ?ｿｽc?ｿｽ?ｿｽ?ｿｽI?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾌハ?ｿｽ?ｿｽ?ｿｽh?ｿｽﾌ開?ｿｽ?ｿｽ?ｿｽ?ｿｽ
	INT32				*stroke_stop;//[mm]
	// ?ｿｽ?ｿｽa?ｿｽc?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽO?ｿｽa?ｿｽc?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽw?ｿｽ閧ｷ?ｿｽ?ｿｽt?ｿｽ?ｿｽ?ｿｽO
	BOOL				*is_close;

	// ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ ?ｿｽz?ｿｽ?ｿｽ?ｿｽﾌゑｿｽ ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ
	// ?ｿｽz?ｿｽ?ｿｽ?ｿｽI?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾌ蛇包ｿｽ?ｿｽ?ｿｽ?ｿｽﾌ趣ｿｽ?ｿｽk?ｿｽ?ｿｽ
	float				*shrink_length;//[mm]
} MBP_GraspResult;

// *************************************
//         ?ｿｽﾖ撰ｿｽ?ｿｽv?ｿｽ?ｿｽ?ｿｽg?ｿｽ^?ｿｽC?ｿｽv?ｿｽ骭ｾ         
// *************************************
	MBP_API MBP_ERR_CODE MBP_AllocSetting(MBP_Setting **setting);
	MBP_API MBP_ERR_CODE MBP_DeleteSetting(MBP_Setting **setting);

	MBP_API MBP_ERR_CODE MBP_AllocSearchResult(MBP_SearchResult **search_result);
	MBP_API MBP_ERR_CODE MBP_DeleteSearchResult(MBP_SearchResult **search_result);

	MBP_API MBP_ERR_CODE MBP_AllocGraspResult(MBP_GraspResult **grasp_result);
	MBP_API MBP_ERR_CODE MBP_DeleteGraspResult(MBP_GraspResult **grasp_result);

	MBP_API MBP_ERR_CODE MBP_AllocMesh(MBP_Mesh **mbp_mesh);
	MBP_API MBP_ERR_CODE MBP_DeleteMesh(MBP_Mesh **mbp_mesh);
	MBP_API MBP_ERR_CODE MBP_AllocMeshes(MBP_Meshes **mbp_meshes);
	MBP_API MBP_ERR_CODE MBP_DeleteMeshes(const INT32 index, MBP_Meshes **mbp_meshes);
	MBP_API MBP_ERR_CODE MBP_DeleteAllMeshes(MBP_Meshes **mbp_meshes);
	MBP_API MBP_ERR_CODE MBP_ReadSTL(const char *cad_filename, MBP_Mesh *mbp_mesh);
	MBP_API MBP_ERR_CODE MBP_ReadSTL2Meshes(const char *cad_filename, const INT32 index, MBP_Meshes *mbp_meshes);

	MBP_API MBP_ERR_CODE MBP_AllocBox(MBP_Box **mbp_box);
	MBP_API MBP_ERR_CODE MBP_DeleteBox(MBP_Box **mbp_box);

	MBP_API MBP_ERR_CODE MBP_TransformSearchResultToBaseCooSys(const MBP_SearchResult *result_cam, const MBP_Setting *setting, const float rot_tool_capture2base[9], const float trans_tool_capture2base[3], MBP_SearchResult *result_base);
	MBP_API MBP_ERR_CODE MBP_ConvertAngleOrder(const float *input_angle, const e_MBP_RotAngleOrder input_order, const INT32 sample_num, float *output_angle, const e_MBP_RotAngleOrder output_order);
	MBP_API char *MBP_GetErrorMessage(const MBP_ERR_CODE error_code);

	MBP_API MBP_ERR_CODE MBP_GetSearchResultSize(const MBP_SearchResult *search_result, INT32 *search_result_size);
	MBP_API MBP_ERR_CODE MBP_CopySearchResult(const MBP_SearchResult *search_result, const INT32 search_result_size, UINT8 *pack_search_result);

#if defined( __cplusplus )
}
#endif


#endif	/* MBP_API_H_ */
