// *********************************************************************
//              (C) Copyright OMRON CORPORATION 2017
//                        All Rights Reserved
// ---------------------------------------------------------------------
//               MBP (Mogra for Bin Picking) プロジェクト
// *********************************************************************
// フ ァ イ ル 名 : mbp_error.h
// 処  理  概  要 : マクロ定義（エラーコード）
// 
// 日付         作成者          内容
// ------------ --------------- ---------------------------------------
// 2017/12/28   Y. Nishina      新規作成
// *********************************************************************

#ifndef MBP_ERROR_H_
#define MBP_ERROR_H_
//#include "basetsd.h"
#include "win_type.h"

// *************************************
//         型      定      義         
// *************************************
typedef INT32 MBP_ERR_CODE;

// *************************************
//         マ  ク  ロ   定   義         
// *************************************
#define MAKE_ERROR_CODE(major,minor)		(((major) << 16) | (minor))			// エラーコード(32bit)生成

// エラーコード詳細
#define MBP_MAJOR_NORMAL					0x0000			// Major：正常終了

#define MBP_ERR_MAJOR_VARIOUS				0x0001			// Major：未定義エラー

#define MBP_ERR_MAJOR_PARAM					0x0002			// Major：パラメータエラー
#define MBP_ERR_MINOR_NOT_INITIALIZED		0x0001			// Minor：未初期化
#define MBP_ERR_MINOR_INVALID_VALUE			0x0002			// Minor：不正な値
#define MBP_ERR_MINOR_NULL					0x0003			// Minor：NULL
#define MBP_ERR_MINOR_ALREADY_SET			0x0004			// Minor：設定済

#define MBP_ERR_MAJOR_FILE					0x0003			// Major：ファイルエラー
#define MBP_ERR_MINOR_CANNOT_OPEN			0x0001			// Minor：ファイルを開けない
#define MBP_ERR_MINOR_INVALID_FORMAT		0x0002			// Minor：フォーマットが不正

#define MBP_ERR_MAJOR_MATH					0x0004			// Major：数学エラー
#define MBP_ERR_MINOR_ZERO_DIVISION			0x0001			// Minor：ゼロ割
#define MBP_ERR_MINOR_LOG_NON_POSITIVE		0x0002			// Minor：非正値の対数ログ
#define MBP_ERR_MINOR_NO_SOLUTION			0x0003			// Minor：解なし

#define MBP_ERR_MAJOR_MEMORY				0x0005			// Major：メモリエラー
#define MBP_ERR_MINOR_ALLOC					0x0001			// Minor：領域確保エラー
#define MBP_ERR_MINOR_ZERO_ALLOC			0x0002			// Minor：領域サイズゼロでメモリ確保

#define MBP_ERR_MAJOR_IMAGE					0x0006			// Major：画像エラー
#define MBP_ERR_MINOR_OUTSIDE_ACCESS		0x0001			// Minor：領域外アクセス

#define MBP_ERR_MAJOR_CAD	    			0x0010			// Major：Cad matchingに関するエラー
#define MBP_ERR_MAJOR_GRASP	    			0x0011			// Major：Grasp planningに関するエラー
#define MBP_ERR_MAJOR_AOS	    			0x0012			// Major：Active one shotに関するエラー
#define MBP_ERR_MAJOR_HANDEYE    			0x0013			// Major：Hand eye calibrationに関するエラー
#define MBP_ERR_MAJOR_ENV					0x0014			// Major：EnvSetupに関するエラー
#define MBP_ERR_MAJOR_MV					0x0015			// Major：MultiViewに関するエラー
#define MBP_ERR_MAJOR_MODELLESS				0x0016			// Major：ModelLessに関するエラー

// エラーコード（32bit）＝ Major code(上位16bit) ＋ Minor code(下位16bit)
#define MBP_NORMAL							MAKE_ERROR_CODE(MBP_MAJOR_NORMAL, 0x0000)

#define MBP_ERR_VARIOUS						MAKE_ERROR_CODE(MBP_ERR_MAJOR_VARIOUS, 0x0000)

#define MBP_ERR_PARAM_NOT_INITIALIZED		MAKE_ERROR_CODE(MBP_ERR_MAJOR_PARAM, MBP_ERR_MINOR_NOT_INITIALIZED)
#define MBP_ERR_PARAM_INVALID_VALUE			MAKE_ERROR_CODE(MBP_ERR_MAJOR_PARAM, MBP_ERR_MINOR_INVALID_VALUE)
#define MBP_ERR_PARAM_NULL					MAKE_ERROR_CODE(MBP_ERR_MAJOR_PARAM, MBP_ERR_MINOR_NULL)
#define MBP_ERR_PARAM_ALREADY_SET			MAKE_ERROR_CODE(MBP_ERR_MAJOR_PARAM, MBP_ERR_MINOR_ALREADY_SET)

#define MBP_ERR_FILE_CANNOT_OPEN			MAKE_ERROR_CODE(MBP_ERR_MAJOR_FILE, MBP_ERR_MINOR_CANNOT_OPEN)
#define MBP_ERR_FILE_INVALID_FORMAT			MAKE_ERROR_CODE(MBP_ERR_MAJOR_FILE, MBP_ERR_MINOR_INVALID_FORMAT)

#define MBP_ERR_MATH_ZERO_DIVIZION			MAKE_ERROR_CODE(MBP_ERR_MAJOR_MATH, MBP_ERR_MINOR_ZERO_DIVISION)
#define MBP_ERR_MATH_LOG_NON_POSITIVE		MAKE_ERROR_CODE(MBP_ERR_MAJOR_MATH, MBP_ERR_MINOR_LOG_NON_POSITIVE)
#define MBP_ERR_MATH_NO_SOLUTION			MAKE_ERROR_CODE(MBP_ERR_MAJOR_MATH, MBP_ERR_MINOR_NO_SOLUTION)

#define MBP_ERR_MEMORY_ALLOC				MAKE_ERROR_CODE(MBP_ERR_MAJOR_MEMORY, MBP_ERR_MINOR_ALLOC)
#define MBP_ERR_MEMORY_ZERO_ALLOC			MAKE_ERROR_CODE(MBP_ERR_MAJOR_MEMORY, MBP_ERR_MINOR_ZERO_ALLOC)

#define MBP_ERR_IMAGE_OUTSIDE_ACCESS		MAKE_ERROR_CODE(MBP_ERR_MAJOR_IMAGE, MBP_ERR_MINOR_OUTSIDE_ACCESS)

#define GET_ERROR_MAJOR(e)				    (((e) & 0x0FFFF0000) >> 16)			// Majorコード取得
#define GET_ERROR_HIGH_MINOR(e)			    (((e) >> 8) & 0x000000FF)			// Minorコード・上位8bit取得
#define GET_ERROR_LOW_MINOR(e)			    ((e) & 0x000000FF)					// Minorコード・下位8bit取得


#endif	/* MBP_ERROR_H_ */