/*****************************************************************************/
/*! @addtogroup 
 *  @file   Kinect.h
 *  @brief  Kinectの処理に関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef	KINECT_H
#define	KINECT_H

// インクルードファイル
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <string>
#include <Windows.h>
#include <NuiApi.h>
#include <NuiKinectFusionApi.h>
#include <opencv2\opencv.hpp>
#include <time.h>

// バージョン名の取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// libファイル名の最後の部分をReleaseとDebugで分ける
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif

// ライブラリファイル
#pragma comment(lib, "Kinect10.lib")
#pragma comment(lib, "KinectFusion180_64.lib")
#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_video" CV_VERSION_STR CV_EXT_STR)

using namespace std;
using namespace cv;

/*! @class  Kinect
 *  @brief  Kinectの処理を行うクラス
 */
class Kinect
{
private:
	INuiSensor	*kinect;	/*!< Kinect */
	HANDLE		h_image;    /*!< カラーカメラのハンドラ */
	HANDLE		h_depth;    /*!< 深度カメラのハンドラ */
	HANDLE		streamEvent;  /*!< イベント */
	HRESULT		result;		/*!< エラーチェック */
	DWORD		width;		/*!< 横幅 */
	DWORD		height;		/*!< 縦幅 */
  float   max_depth;  /*!< 深度情報の最大値 */

public:
	Kinect(void);
	~Kinect(void);
  bool  initialize(float max_depth = 8192.0);
  bool  colorImage(Mat &img);
  bool  depthImage(Mat &img);
  bool  depthSave(string name);
  bool  depthImageXZ(Mat &img);
  bool  depthImageYZ(Mat &img);
  void  createImageData(float fps = 5.0);
  void  createCorrectImageData(float fps = 5.0);
  void  createDepthImage(char *name);

  string  int2String(int num);
};

#endif