/*****************************************************************************/
/*! @addtogroup 
 *  @file   FileOperation.h
 *  @brief  ファイル操作に関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/
#ifndef FILEOPERATION_H
#define FILEOPERATION_H

#include <time.h> // 処理時間計測用
#include <direct.h> // フォルダ作成用
#include <fstream>
#include <istream>
#include "../Miscellaneous/Misc.h"
#include "WriteNPThread.h"

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

/* 定数 */
#define MAX_LABEL   8000  
#define MAX_FEATURE 300   

/*! @namespace FileOperation
 *  @brief  ファイル操作用の名前空間
 */
namespace FileOperation
{
  // 書き込み処理
  void writeLabelDat(const Mat &label, const int num);
  void writeFeaturesDat(const Features &features, const int num);
  void writeSceneFlowDat(const Mat &flow, const int num);

  /*vector<FeatureSequence> writeSubRegionSequencesDat(TemporalSubRegion &region, vector<Features> &f_vec,
    vector<Mat> &labels, const int N, const int start, const int end, 
    const int min_frame, const float min_move, bool flag);*/

  void writeRegionSequencesDat(RegionSequences &region_seq, const int start, const int end);
  void writeNearestPointDat(RegionSequences &region_seq, vector<Mat> &labels, 
    const int N, const int start, const int end);
  
  // 読み込み処理
  Mat readLabelDat(const int num, const int width, const int height);
  vector<Mat> readMultipleLabelDat(const int start, const int end, const int width,
    const int height);
  Features readFeaturesDat(const int num);
  vector<Features> readMultipleFeaturesDat(const int start, const int end);
  Mat readSceneFlowDat(const int num, const int width, const int height);
  vector<Mat> readMultipleSceneFlowDat(const int start, const int end, 
    const int width, const int height);

  //vector<FeatureSequence> readMultipleSequencesDat(void);
  //vector<FeatureSequence> readMultipleSequencesDat(const int start, const int end);
  RegionSequences readRegionSequencesDat(const int start, const int end);
  
  //Mat readDistanceDat(const int num, const int subregion_num);
  //vector<Mat> readMultipleDistanceDat(const int start, const int end, 
  //  const int subregion_num);

  //void readDistancePointDat(const int num, const int x, const int y,
  //  vector<Point> &pt1, vector<Point> &pt2);
  void readDistancePointDat(const int num, const int x, const int y,
    vector<Point> &pt1, vector<Point> &pt2, const int start, const int end);
}

#endif