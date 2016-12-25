/*****************************************************************************/
/*! @addtogroup 
 *  @file   GraphBased.cpp
 *  @brief  部分領域への分割（抽出）に関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "GraphBased.h"

/**
 * @brief コンストラクタ
 * @param[in] k         部分領域の抽出閾値
 * @param[in] min_size  最小の部分領域サイズ
 * @param[in] w_I       画像情報の重み係数
 * @param[in] w_D       深度情報の重み係数
 * @param[in] w_F       運動情報の重み係数
 */
GraphBased::GraphBased(float k, int min_size, float w_I, float w_D, float w_F)
{
  this->k = k;
  this->min_size = min_size;
  this->w_I = w_I;
  this->w_D = w_D;
  this->w_F = w_F;
}

/**
 * @brief デストラクタ
 */
GraphBased::~GraphBased(void)
{

}

/**
 * @brief 画像の領域分割
 * @param[in] color 画像情報
 * @param[in] depth 深度情報
 * @param[in] flow  運動情報
 * @return  部分領域の抽出結果
 */
SubRegion GraphBased::segmentImage(Mat color, Mat depth, Mat flow)
{
  width = color.cols;
  height = color.rows;

  // グラフの構築
  vector<edge> edges;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      edge e;

      // 右
      if (x < width - 1) {
        e.a = y * width + x;
        e.b = y * width + (x + 1);
        e.w = diff(color, depth, flow, Point(x, y), Point(x + 1, y));
        edges.push_back(e);
      }

      // 下
      if (y < height - 1) {
        e.a = y * width + x;
        e.b = (y + 1) * width + x;
        e.w = diff(color, depth, flow, Point(x, y), Point(x, y + 1));
        edges.push_back(e);
      }

      // 右下
      if ((x < width - 1) && (y < height - 1)) {
        e.a = y * width + x;
        e.b = (y + 1) * width + (x + 1);
        e.w = diff(color, depth, flow, Point(x, y), Point(x + 1, y + 1));
        edges.push_back(e);
      }

      // 右上
      if ((x < width -1) && (y > 0)) {
        e.a = y * width + x;
        e.b = (y - 1) * width + (x + 1);
        e.w = diff(color, depth, flow, Point(x, y), Point(x + 1, y - 1));
        edges.push_back(e);
      }
    }
  }

  // 部分領域の抽出
  SubRegion region = segmentGraph(edges);

  // 微小領域の統合
  int num_edges = edges.size();
  for (int i = 0 ; i < num_edges; i++) {
    int a = region.find(edges.at(i).a);
    int b = region.find(edges.at(i).b);
    if ((a != b) && ((region.size(a) < min_size) || (region.size(b) < min_size))) {
      region.join(a, b);
    }
  }

  return region;
}

/**
 * @brief 画素間の相違度の計算
 * @param[in] color 画像情報（RGBの輝度値）
 * @param[in] depth 深度情報
 * @param[in] flow  運動情報
 * @param[in] p1  画素1
 * @param[in] p2  画素2
 * @return  画素間の特徴量の相違度（Sum of Squared Difference）
 */
float GraphBased::diff(Mat color, Mat depth, Mat flow, Point p1, Point p2)
{
  float weight;
  Vec3f cf1 = color.at<Vec3f>(p1);
  Vec3f cf2 = color.at<Vec3f>(p2);
  float df1 = depth.at<float>(p1);
  float df2 = depth.at<float>(p2);
  Vec3f ff1 = flow.at<Vec3f>(p1); // 運動情報の特徴xyz
  Vec3f ff2 = flow.at<Vec3f>(p2); // 運動情報の特徴xyz

  float diff_i = w_I * sqrtf((square(cf1[0] - cf2[0]) + square(cf1[1] - cf2[1])
      + square(cf1[2] - cf2[2])));
  float diff_d = w_D * sqrtf(square(df1 - df2));
  float diff_f = w_F * sqrtf((square(ff1[0] - ff2[0]) + square(ff1[1] - ff2[1])
      + square(ff1[2] - ff2[2])));

  //// 深度画像のノイズが検出された場合
  //if (feature1[3] <= 0 || feature2[3] <= 0) {
  //  // 輝度と運動のみで重みを計算
  //  weight = diff_i + diff_f;
  //}
  //else {
    weight = diff_i + diff_d + diff_f;
//  }

  return weight;
}

/**
 * @brief グラフのセグメンテーション
 * @param[in] edges     エッジのリスト
 * @return 部分領域の抽出結果(最少の部分領域含む)
 */
SubRegion GraphBased::segmentGraph(vector<edge> edges)
{
  int num_edges = edges.size();

  SubRegion u(width, height);

  // 重み順にエッジをソート
  sort(edges.begin(), edges.end());

  // 閾値の初期化
  int num_vertices = width * height;
  vector<float> threshold(num_vertices);
  for (int i = 0; i < num_vertices; i++) {
    threshold[i] = THRESHOLD(1, k);
  }

  // 重み順に処理を実行
  for (int i = 0; i < num_edges; i++) {
    edge &pedge = edges.at(i);

    // 部分領域（ノード）の統合
    int a = u.find(pedge.a);
    int b = u.find(pedge.b);

    if (a != b) {
      if ((pedge.w <= threshold[a]) && (pedge.w <= threshold[b])) {
        u.join(a, b);
        a = u.find(a);
        threshold[a] = pedge.w + THRESHOLD(u.size(a), k);
      }
    }
  }

  return u;
}