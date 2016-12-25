/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainCreateVideo.cpp
 *  @brief  実験データ（画像・深度情報の系列）の生成プログラムに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/
#include <direct.h> // フォルダ作成用
#include <sys\stat.h>
//#include "Kinect.h"
#include "../Kinect/Kinect.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/Kinectd.lib")
#else
#pragma comment(lib, "../x64/Release/Kinect.lib")
#endif

/**
 * @brief メイン関数
 */
int main(int argc, char *argv[])
{
  _mkdir("video");

  // データセットの取得
  Kinect kinect;
  if (!kinect.initialize(3000.0)) {
    return -1;
  }
  kinect.createImageData(30.0);
//  kinect.createCorrectImageData(30.0);

  return 0;
}