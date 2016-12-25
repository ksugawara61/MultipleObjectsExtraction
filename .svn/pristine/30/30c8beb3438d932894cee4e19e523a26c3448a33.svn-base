/*****************************************************************************/
/*! @addtogroup 
 *  @file   TemporalGraphBased.cpp
 *  @brief  時間方向に安定した部分領域への分割（抽出）処理に関するファイル
 *          ※ビルド時エラー発生
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "TemporalGraphBased.h"

/**
 * @brief コンストラクタ
 * @param[in] sigma     ガウス分布の標準偏差
 * @param[in] aperture  フィルタの窓サイズ
 * @param[in] k         部分領域の抽出閾値
 * @param[in] min_size  最小の部分領域サイズ
 * @param[in] w_I
 * @param[in] w_D
 * @param[in] w_F
 * @param[in] color_sigma
 * @param[in] iteration
 * @param[in] rs
 * @param[in] rd
 * @param[in] alpha
 * @param[in] beta
 * @param[in] gamma
 */
TemporalGraphBased::TemporalGraphBased(float sigma, int aperture, float k, int min_size,
  float w_I, float w_D, float w_F, float color_sigma, int iteration, float rs, float rd, 
  float alpha, float beta, float gamma)
{
  this->space_sigma = sigma;
  this->aperture = aperture;
  this->k = k;
  this->min_size = min_size;
  this->w_I = w_I;
  this->w_D = w_D;
  this->w_F = w_F;
  this->color_sigma = color_sigma;
  this->iteration = iteration;
  this->rs = rs;
  this->rd = rd;
  this->alpha = alpha;
  this->beta = beta;
  this->gamma = gamma;
}

/**
 * @brief デストラクタ
 */
TemporalGraphBased::~TemporalGraphBased(void)
{

}

/**
 * @brief 前のフレーム結果を用いた画像の領域分割
 * @param[in] c_curr 画像情報
 * @param[in] d_curr 深度情報
 * @param[in] c_past 前のフレームの画像情報
 * @param[in] d_past 前のフレームの深度情報
 * @param[in] tag    前のフレームの分割結果（ラベル）
 * @return  部分領域
 */
SubRegion TemporalGraphBased::segmentCorrectImage(Mat c_curr, Mat d_curr,
  Mat c_past, Mat d_past, Mat tag)
{
  width = c_curr.cols;
  height = d_curr.rows;

  // チャンネル毎に平滑化
  Mat gray_curr = smoothing(c_curr, d_curr);
  Mat gray_past = smoothing(c_past, d_past);

  // グラフの構築
  vector<edge> edges;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      edge e;

      // 右
      if (x < width - 1) {
        e.a = y * width + x;
        e.b = y * width + (x + 1);
        e.w = diff(gray_curr, Point(x, y), Point(x + 1, y));
        edges.push_back(e);
      }

      // 下
      if (y < height - 1) {
        e.a = y * width + x;
        e.b = (y + 1) * width + x;
        e.w = diff(gray_curr, Point(x, y), Point(x, y + 1));
        edges.push_back(e);
      }

      // 右下
      if ((x < width - 1) && (y < height - 1)) {
        e.a = y * width + x;
        e.b = (y + 1) * width + (x + 1);
        e.w = diff(gray_curr, Point(x, y), Point(x + 1, y + 1));
        edges.push_back(e);
      }

      // 右上
      if ((x < width -1) && (y > 0)) {
        e.a = y * width + x;
        e.b = (y - 1) * width + (x + 1);
        e.w = diff(gray_curr, Point(x, y), Point(x + 1, y - 1));
        edges.push_back(e);
      }
    }
  }

  // フレーム間での重みの平均
  m_past = aveDiffTemporal(gray_curr, gray_past);
  m_past = square(m_past);

  // 部分領域の抽出
  SubRegion region = segmentCorrectGraph(width, height, edges, gray_curr,
    gray_past, tag);

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
 * @brief 累積境界の計算
 * @param[in] boundarys 境界のリスト
 */
void TemporalGraphBased::comulativeBoundary(const vector<Mat> &boundarys)
{
  int frame_num = boundarys.size();
  if (frame_num != 0) {
    width = boundarys.at(0).cols;
    height = boundarys.at(0).rows;
  }

  comboundary = Mat::zeros(height, width, CV_32FC1);

  // 分子の計算
  float numerator = 0.0;
  for (int t = 0; t < frame_num; t++) {
    numerator += exp(-1 * (t + 1) / TC);
  }

  // 分母の計算
  for (int i = 0; i < width * height; i++) {
    for (int t = 0; t < frame_num; t++) {
      if (boundarys.at(t).at<uchar>(i) != 0) {
        comboundary.at<float>(i) += exp(-1 * (t + 1) / TC);
      }
    }
  }

  // 値の計算
  for (int i = 0; i < width * height; i++) {
    comboundary.at<float>(i) /= numerator;
  }
}

/**
 *
 */
void TemporalGraphBased::useFlow(const Mat &flow)
{
  this->flow = flow.clone();
}

/**
 * @brief グラフのセグメンテーション
 * @param[in] width   横幅
 * @param[in] height  縦幅
 * @param[in] edges   エッジ
 * @param[in] gray_curr 現在のフレームの特徴
 * @param[in] gray_past 前のフレームの特長
 * @param[in] tag       前のフレームのラベル
 * @return u  部分領域
 */
SubRegion TemporalGraphBased::segmentCorrectGraph(int width, int height, 
  vector<edge> edges, const Mat &gray_curr, const Mat &gray_past,
  const Mat &tag)
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

  // 信頼度，信頼度の総和，タグの初期化
  vector<float> c(num_vertices);
  vector<float> sum(num_vertices);
  vector<ConfList> c_list(num_vertices);
  for (int i = 0; i < num_vertices; i++) {
    c[i] = conf(gray_curr, gray_past, Point(i % width, i / width));
    sum[i] = c[i];

    c_list.at(i).initialize(tag.at<int>(i), c[i]);
  }

  // 新機能
  // 動きのある画素のタグを更新
  if (!flow.empty()) {
    Point3f vel;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        vel = flow.at<Point3f>(y, x);
        int index = (y + vel.y) * width + x + vel.x;
        if (index < num_vertices && index >= 0) {
          c_list.at(index).initialize(tag.at<int>(y, x), c[y * width + x]);
        }
      }
    }
  }

  // 領域間相違性の補正パラメータ
  float b = exp(-1 * m_past / beta);

  // 重み順に処理を実行
  for (int i = 0; i < num_edges; i++) {
    edge &pedge = edges.at(i);

    // 部分領域（ノード）の統合
    int x = u.find(pedge.a);
    int y = u.find(pedge.b);

    if (x != y) {
      // 補正項の計算
      float f;
      if (c_list.at(x).argmaxConf() == c_list.at(y).argmaxConf()) {
        f = 1 / pow(1 - (c[x] * c[y]), rs);
      }
      else {
        f = 1 - pow(1 - (c[x] * c[y]), rd);
      }

      // 補正係数の計算
      float g = 1.0;
      if (!comboundary.empty()) {
        // 累積境界値の平均
        float v = 0.0;
     
        int mini = min(pedge.a, pedge.b);
        int maxi = max(pedge.a, pedge.b);

        // （横並びの時）
        if (abs(pedge.a - pedge.b) == 1) {
          if (mini / width != 0) {
            v += comboundary.at<float>(mini - width);
          }
          else {
            v += comboundary.at<float>(mini);
          }

          v += comboundary.at<float>(mini);
          
          if (mini / width != height - 1) {
            v += comboundary.at<float>(mini + width);
          }
          else {
            v += comboundary.at<float>(mini);
          }

          if (maxi / width != 0) {
            v += comboundary.at<float>(maxi - width);
          }
          else {
            v += comboundary.at<float>(maxi);
          }
          
          v += comboundary.at<float>(maxi);
          
          if (maxi / width != height - 1) {
            v += comboundary.at<float>(maxi + width);
          }
          else {
            v += comboundary.at<float>(maxi);
          }

          v /= 6;
        }
        // （縦並びの時）
        else if (abs(pedge.a - pedge.b) == width) {
          if (mini % width != 0) {
            v += comboundary.at<float>(mini - 1);
          }
          else {
            v += comboundary.at<float>(mini);
          }

          v += comboundary.at<float>(mini);
          
          if (mini % width != width - 1) {
            v += comboundary.at<float>(mini + 1);
          }
          else {
            v += comboundary.at<float>(mini);
          }

          if (maxi % width != 0) {
            v += comboundary.at<float>(maxi - 1);
          }
          else {
            v += comboundary.at<float>(maxi);
          }
          
          v += comboundary.at<float>(maxi);
          
          if (maxi % width != width - 1) {
            v += comboundary.at<float>(maxi + 1);
          }
          else {
            v += comboundary.at<float>(maxi);
          }

          v /= 6;
        }
        // （右または左斜め並びの時）
        else if (abs(pedge.a - pedge.b) == width + 1 
          || abs(pedge.a - pedge.b) == width - 1) 
        {
          v += comboundary.at<float>(mini);
          v += comboundary.at<float>(mini + width);
          v += comboundary.at<float>(maxi - width);
          v += comboundary.at<float>(maxi);

          v /= 4;
        }

        g = (-1 * b * (1 - v) + (1 + b)) *
          (-1 * b * exp(-1 * alpha * pedge.w) + (1 + b)) - b;
      }

      if ((pedge.w * g <= threshold[x] * f) && (pedge.w * g <= threshold[y] * f)) {
        // 信頼度の総和の更新
        sum[x] += c[y];
        sum[y] += c[x];

        // タグの更新
        c_list.at(x).integrate(c_list.at(y));
        
        // 統合
        u.join(x, y);
        x = u.find(x);
        threshold[x] = pedge.w + THRESHOLD(u.size(x), k);

        // 信頼度の更新
        c[x] = sum[x] / u.size(x);
      }
    }
  }

  return u;
}

/**
 * @brief 信頼度の計算
 * @param[in] gray_curr 現在のフレームの輝度・深度値
 * @param[in] gray_past 前のフレームの輝度・深度値
 * @param[in] p 注目画素
 * @return  注目画素の信頼度
 */
float TemporalGraphBased::conf(const Mat &gray_curr, const Mat &gray_past, Point p)
{
  float w_past = gamma * square(diffTemporal(gray_curr, gray_past, p));
  
  return exp(-1 * (w_past / m_past));
}

/**
 * @brief フレーム間での注目画素の重みを計算
 * @param[in] gray_curr 現在のフレームの輝度・深度値
 * @param[in] gray_past 前のフレームの輝度・深度値
 * @param[in] p 注目画素
 * @return  重みw_past(p)
 */
float TemporalGraphBased::diffTemporal(const Mat &gray_curr, const Mat &gray_past, Point p)
{
  float weight;
  Vec4f feature1 = gray_curr.at<Vec4f>(p);
  Vec4f feature2 = gray_past.at<Vec4f>(p);

  // 深度画像のノイズが検出された場合
  if (feature1[3] <= 0 || feature2[3] <= 0) {
    // 輝度値のみで重みを計算
    weight = sqrtf((square(feature1[0] - feature2[0]) + square(feature1[1] - feature2[1])
      + square(feature1[2] - feature2[2])));
  }
  else {
    weight = sqrtf(square(feature1[0] - feature2[0]) + square(feature1[1] - feature2[1])
      + square(feature1[2] - feature2[2]) + (square(feature1[3] - feature2[3]) * 3.0));
  }

  return weight;
}

/**
 * @brief フレーム間での画像全体の重みの平均の計算
 * @param[in] gray_curr 現在のフレームの輝度・深度値
 * @param[in] gray_past 前のフレームの輝度・深度値
 * @return  フレーム間での画像全体の重みの平均
 */
float TemporalGraphBased::aveDiffTemporal(const Mat &gray_curr, const Mat &gray_past)
{
  float average = 0.0;
  int width = gray_curr.cols;
  int height = gray_curr.rows;

  // エッジの重みの総和を計算
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      average += diffTemporal(gray_curr, gray_past, Point(x, y));
    }
  }

  // エッジの重みの平均値を計算
  average /= width * height;

  return average;
}