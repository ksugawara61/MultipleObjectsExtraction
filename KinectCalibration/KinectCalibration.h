/* Kinect.h */
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
#include "../GraphBased/GraphBased.h"

// バージョン名の取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// libファイル名の最後の部分をReleaseとDebugで分ける
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#pragma comment(lib, "../x64/Debug/GraphBasedd.lib")
#else
#define CV_EXT_STR ".lib"
#pragma comment(lib, "../x64/Release/GraphBased.lib")
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

class Kinect	// Kinect処理用クラス
{
private:
	INuiSensor	*kinect;	// Kinect
	HANDLE		h_image;
	HANDLE		h_depth;
	HANDLE		streamEvent;
	HRESULT		result;		// エラーチェック
	DWORD		width;		// 幅
	DWORD		height;		// 高さ
  float   max_depth;
  vector<Point3f> realpointsmm;
  vector<Point> centroids;
  Mat label;
  Point3f prevpoint;

  static void onMouse(int event, int x, int y, int flags, void *param);
  void read3DCoordinates(int x, int y);
  void readCentroid3DCoordinates(int x, int y);
  int preprocess(Mat &label, SubRegion &region);
  Mat drawSubRegion(Mat &label);
  void calcCentroid(const Mat &label, const int region_num);

public:
	Kinect(void);
	~Kinect(void);
  bool  initialize(float max_depth = 8192.0);
  void  colorAndDepthImage();
};

#endif