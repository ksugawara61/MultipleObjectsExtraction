/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionExtraction.h
 *  @brief  部分領域の抽出処理に関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef SUBREGIONEXTRACTION_H
#define SUBREGIONEXTRACTION_H

#include "../MotionVector/MotionVector.h"
#include "../GraphBased/GraphBased.h"
//#include "../TemporalGraphBased/TemporalGraphBased.h"
#include "../Miscellaneous/Misc.h"
#include "../FileOperation/FileOperation.h"
#include "../DrawRegion/DrawRegion.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/GraphBasedd.lib")
//#pragma comment(lib, "../x64/Debug/TemporalGraphBasedd.lib")
#pragma comment(lib, "../x64/Debug/MotionVectord.lib")
#pragma comment(lib, "../x64/Debug/Miscellaneousd.lib")
#pragma comment(lib, "../x64/Debug/FileOperationd.lib")
#pragma comment(lib, "../x64/Debug/DrawRegiond.lib")
#else
#pragma comment(lib, "../x64/Release/GraphBased.lib")
//#pragma comment(lib, "../x64/Release/TemporalGraphBased.lib")
#pragma comment(lib, "../x64/Release/MotionVector.lib")
#pragma comment(lib, "../x64/Release/Miscellaneous.lib")
#pragma comment(lib, "../x64/Release/FileOperation.lib")
#pragma comment(lib, "../x64/Release/DrawRegion.lib")
#endif

using namespace DrawRegion;

/*! @class  SubRegionExtraction
 *  @brief  部分領域の抽出処理に関するクラス
 */
class SubRegionExtraction
{
public:
  SubRegionExtraction(const float sigma = 0.5, const int ap_filter = 3, const float k = 500,
    const int min_size = 20, const float w_I = 1.0, const float w_D = 3.0, const float w_F = 0.1,
    const float color_sigma = 0, const int smooth_it = 1,
    const int ap_flow = 15, const int levels = 3,
    const int iterations = 3, const float scale = 0.5);
  ~SubRegionExtraction(void);
  Features subRegionExtract(Mat &label, const int num, Mat c_prev, 
    Mat d_prev, const Mat &c_next, const Mat &d_next, vector<Mat> &boundarys);
  //Features correctSubRegionExtract(Mat &label, const int num, const Mat &c_prev, 
  //  const Mat &d_prev, const Mat &c_curr, const Mat &d_curr,
  //  const Mat &c_next, const Mat &d_next, vector<Mat> &boundarys); 

private:
  void smoothing(Mat &color, Mat &depth);
  int preprocess(Mat &label, Mat &boundary, SubRegion &region);
  Features calcFeatures(Mat &label, const int frame_num, const int region_num, 
    const Mat &color, const Mat &depth, const Mat &flow, const Mat &boundary);
  float histgramMode(const vector<int> &histgram, const int max, 
    const int min, const float interval);

  // 部分領域抽出のパラメータ
  float sigma;  /*!<　空間のガウス分布の標準偏差 */
  int ap_filter;  /*!< フィルタの窓サイズ */
  float k;        /*!< 部分領域の抽出閾値 */
  int min_size;   /*!< 最小領域サイズ */
  float w_I;      /*!< 画像情報の重み係数 */
  float w_D;      /*!< 深度情報の重み係数 */
  float w_F;      /*!< 運動情報の重み係数 */
  float color_sigma;  /*!< 輝度のガウス分布の標準偏差 */
  int smooth_it;      /*!< 平滑化の反復回数 */
  int ap_boundary;     /*!< 部分領域の境界と判定する範囲 */

  // シーンフロー推定のパラメータ
  int ap_flow;    /*!< フローの計算に用いる窓サイズ */
  int levels;     /*!< フローの画像ピラミッドのレベル */
  int iterations; /*!< フローの計算アルゴリズムの反復回数 */
  float scale;    /*!< 画像ピラミッドのスケールサイズ */
};

///////////////////////////////////////////////////////////////////////////////
// 描画関数
void outOpticalFlowDat(const Mat &flow, /*const Mat &boundary, const int aperture, */
  const int num, const int interval);
void outSceneFlowDat(const Mat &depth, const Mat &flow, const int num, const int interval);
///////////////////////////////////////////////////////////////////////////////

#endif