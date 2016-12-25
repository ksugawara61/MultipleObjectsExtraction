/*****************************************************************************/
/*! @addtogroup 
 *  @file   Smooth.h
 *  @brief  画像・深度情報の平滑化ライブラリに関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef SMOOTH_H
#define SMOOTH_H

#include <algorithm>
#include <opencv2\opencv.hpp>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// バージョン名の取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// libファイル名の最後の部分をReleaseとDebugで分ける
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif

#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)

#endif

using namespace cv;

#define SQUARE
template <class T>
inline T square(const T &x) {
  return x * x;
};

/*! @namespace  Smooth
 *  @brief  画像・深度情報の平滑化用名前空間
 */
namespace Smooth
{
  void medianFilterForKinect(const Mat src, Mat &dst, int aperture, uchar outlinere);
  void gaussianFilter(const Mat src, Mat &dst, int aperture, float sigma);
  void gaussianFilter(const Mat src, Mat &dst, int aperture, 
    float sigma, float outlier);
  void bilateralFiler(const Mat src, Mat &dst, int aperture,
    float sigma_color, float sigma_space, float outler = -1);
};

#endif