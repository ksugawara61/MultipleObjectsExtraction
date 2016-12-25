/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionIntegration.h
 *  @brief  領域系列の統合処理に関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef SUBREGIONINTEGRATION_H
#define SUBREGIONINTEGRATION_H

#include "../GraphBased/GraphBased.h"
#include "../Miscellaneous/Misc.h"
#include "../FileOperation/FileOperation.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/GraphBasedd.lib")
#pragma comment(lib, "../x64/Debug/Miscellaneousd.lib")
#pragma comment(lib, "../x64/Debug/FileOperationd.lib")
#else
#pragma comment(lib, "../x64/Release/GraphBased.lib")
#pragma comment(lib, "../x64/Release/Miscellaneous.lib")
#pragma comment(lib, "../x64/Release/FileOperation.lib")
#endif

using namespace FileOperation;

/*! @class  SubRegionIntegration
 *  @brief  領域系列の統合処理に関するクラス
 */
class SubRegionIntegration
{
public:
  SubRegionIntegration(const float threshold, const float w_locate, const float w_flow,
    const int width, const int height, const int start, const int end, bool near_flag,
    const int min_contact);
  ~SubRegionIntegration(void);
  RegionSequences integrateSequence(RegionSequences sequences, const vector<Mat> &flows);

private:
  float sequencesDistance(const vector<Mat> &flows, 
    const vector<RegionSequence> &s, const int x, const int y);
  float subregionsDistance(const Mat &flow, const vector<Point> &pt1, 
    const vector<Point> &pt2, const int num);

  // 部分領域の特徴に基づき距離を計算
  float subregionBasedDistance(const vector<Point> &pt1, 
    const vector<Point> &pt2, const Features f1, const Features f2, const int num);

  bool contactDetection(RegionSequence seq1, RegionSequence seq2, const vector<Mat> &flows);

  //// Cosine類以度での計算
  //float sequenceCosineSimilarity(const vector<Mat> &flows, 
  //  const vector<FeatureSequence> &s, const int x, const int y);
  //float cosineSimilarity(const Mat &flow, const vector<Point> &pt1, 
  //  const vector<Point> &pt2, const int num);

  //// 領域間の位置の類以度
  //float sequenceLocation(const vector<FeatureSequence> &s, const int x, const int y);
  //float subregionLocation(const vector<Point> &pt1, const vector<Point> &pt2, const int num);

  //// 領域間のシーンフローの類以度
  //float sequenceSceneFlow(const vector<Mat> &flows, 
  //  const vector<FeatureSequence> &s, const int x, const int y);
  //float subregionSceneFlow(const Mat &flow, const vector<Point> &pt1, 
  //  const vector<Point> &pt2, const int num);

  //float averageDistance(const vector<Mat> &distmat, const int x, const int y);
  //float stdevDistance(const vector<Mat> &distmat, const int x, const int y);
  //float weightSequences(const FeatureSequence &f1, const FeatureSequence &f2);


  float threshold;    // 領域系列間の相違度の閾値
  int min_contact;    // 領域系列の最少接触回数
  float w_locate; // 位置の重み
  float w_flow;   // シーンフローの重み
  int width;
  int height;
  int start;    // 先頭のフレーム番号
  int end;      // 末尾のフレーム番号

  bool near_flag;  // 最近点の対を利用するか
};

#endif