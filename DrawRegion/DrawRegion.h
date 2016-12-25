/*****************************************************************************/
/*! @addtogroup 
 *  @file   DrawRegion.h
 *  @brief  描画処理用ライブラリに関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/
#ifndef DRAWREGION_H
#define DRAWREGION_H

#include <opencv2\opencv.hpp>
#include "../Miscellaneous/Misc.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// バージョン名の取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// libファイル名の最後の部分をReleaseとDebugで分ける
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#pragma comment(lib, "../x64/Debug/Miscellaneousd.lib")
#else
#define CV_EXT_STR ".lib"
#pragma comment(lib, "../x64/Release/Miscellaneous.lib")
#endif

#endif

using namespace cv;

/*! @namespace DrawRegion
 *  @brief  描画処理用の名前空間
 */
namespace DrawRegion
{
  Mat drawSubRegion(const Mat &label);
  Mat drawSubRegionFeature(const Mat &label, const Mat &boundary, const Features &f,
    const int interval, const float norm);
  Mat drawSubRegionAverage(const Mat &label, const Mat &boundary, const Features &f);

  Mat drawOpticalFlow(const Mat &src, const Mat &flow, const Mat &boundary, 
    const int interval, const int aperture);

  Mat drawMovingSubRegion(const Mat &label, const Features &f, const float norm);
  Mat drawRegionSequence(Features f, //const RegionSequence &region_seq, 
    const Mat &label, const vector<Vec3b> &rgb, const int t);
  Mat drawBoundary(const Mat &label, const int rad);

  //Mat drawRegionSequence(const RegionSequences &region_seq, 
  //  const Mat &label, const vector<Vec3b> &rgb, const int t);
}

#endif