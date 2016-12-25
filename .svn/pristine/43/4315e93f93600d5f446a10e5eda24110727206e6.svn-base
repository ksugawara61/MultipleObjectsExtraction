/*****************************************************************************/
/*! @addtogroup 
 *  @file   GraphBased.h
 *  @brief  部分領域への分割（抽出）に関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef GRAPHBASED_H
#define GRAPHBASED_H

#include <algorithm>
#include <opencv2\opencv.hpp>
#include "disjoint-set.h"
#include "../Smooth/Smooth.h"

// libファイル名の最後の部分をReleaseとDebugで分ける
#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/Smoothd.lib")
#else
#pragma comment(lib, "../x64/Release/Smooth.lib")
#endif

/* 閾値関数の定義 */
#define THRESHOLD(size, k) (k/size) /*!< 閾値関数 */
 
/*! @struct edge
 *  @brief  エッジの構造体
 */
struct edge {
  float w;  /*!< エッジ重み */
  int a; /*!< ノードa */
  int b; /*!< ノードb */

  friend bool operator<(const edge &a, const edge &b) {
    return a.w < b.w;
  }
};

/*! @class  GraphBased
 *  @brief  部分領域への分割（抽出）を行うクラス
 *
 *  P.F. Felzenswalb and D. P. Huttenlocher
 *  "Efficient graph-based image segmentation"
 *  IJCV, Vol.59, No.2, pp.167-181 (2004).
 *  の論文とプログラムを基に実装
 */
class GraphBased
{
public:
  GraphBased(float k = 500, int min_size = 20, 
    float w_I = 1.0, float w_D = 3.0, const float w_F = 0.1);
  ~GraphBased(void);
  SubRegion segmentImage(Mat color, Mat depth, Mat flow);

protected:
//  Mat smoothing(Mat &color, Mat &depth);
  float diff(Mat color, Mat depth, Mat flow, Point p1, Point p2);
  SubRegion segmentGraph(vector<edge> edges);

  float k;      /*!< 部分領域の抽出閾値 */
  int min_size; /*!< 最小領域サイズ */
  float w_I;    /*!< 画像情報の重み係数 */
  float w_D;    /*!< 深度情報の重み係数 */
  float w_F;    /*!< 運動情報の重み係数 */

  int width;
  int height;
};

#endif
