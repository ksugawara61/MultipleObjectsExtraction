/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionExtraction.cpp
 *  @brief  部分領域の抽出処理に関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "SubRegionExtraction.h"

/**
 * @brief コンストラクタ
 * @param[in] sigma     空間のガウス分布の標準偏差
 * @param[in] ap_filter フィルタの窓サイズ
 * @param[in] k         部分領域の統合の閾値
 * @param[in] min_size  部分領域の最小サイズ
 * @param[in] w_I       画像情報の重み係数
 * @param[in] w_D       深度情報の重み係数
 * @param[in] w_F       運動情報の重み係数
 * @param[in] color_sigma 輝度のガウス分布の標準偏差
 * @param[in] smooth_it 平滑化の反復回数
 * @param[in] ap_flow   フローの計算に用いる窓サイズ
 * @param[in] levels    フローの画像ピラミッドのレベル
 * @param[in] iterations  フローの計算アルゴリズムの反復回数
 * @param[in] scale     画像ピラミッドのスケールサイズ
 */
SubRegionExtraction::SubRegionExtraction(const float sigma, const int ap_filter,
  const float k, const int min_size, const float w_I, const float w_D, const float w_F,
  const float color_sigma, const int smooth_it, 
  const int ap_flow, const int levels, const int iterations, const float scale)
{
  this->sigma = sigma;
  this->ap_filter = ap_filter;
  this->k = k;
  this->min_size = min_size;
  this->w_I = w_I;
  this->w_D = w_D;
  this->w_F = w_F;
  this->color_sigma = color_sigma;
  this->smooth_it = smooth_it;
  this->ap_flow = ap_flow;
  this->levels = levels;
  this->iterations = iterations;
  this->scale = scale;
  ap_boundary = ap_filter * 2;  // 後で修正したほうが良い
}

/**
* @brief デストラクタ
*/
SubRegionExtraction::~SubRegionExtraction(void)
{
}

/**
* @brief 部分領域の抽出
* @param[out]  label 部分領域のラベル
* @param[in] frame_num フレーム番号
* @param[in] c_curr  画像情報（対象フレーム）
* @param[in] d_curr  深度情報（対象フレーム）
* @param[in] c_next  画像情報（次のフレーム）
* @param[in] d_next  深度情報（次のフレーム）
* @param[out] boundarys  部分領域の境界のリスト
* @return  対象フレームの特徴量
*/
Features SubRegionExtraction::subRegionExtract(Mat &label, const int frame_num,
  Mat c_curr, Mat d_curr, const Mat &c_next, const Mat &d_next,
  vector<Mat> &boundarys)
{
  // シーンフローの推定
  Mat flow;
  SceneFlow::sceneFlowFarneback(c_curr, d_curr, c_next, d_next, flow, scale,
    levels, ap_flow, iterations, ap_flow / pow((float)(1 / scale), (float)(levels - 1)), 1,1);
  //SceneFlow::sceneFlowLK(c_curr, d_curr, c_next, d_next, flow, Size(15, 15));
  //SceneFlow::sceneFlowSimple(c_curr, d_curr, c_next, d_next, flow);

  //outOpticalFlowDat(flow, frame_num, 3);
  //outSceneFlowDat(d_curr, flow, frame_num, 3);
  
  // シーンフローの3次元化とファイル出力
  Mat flow3d = flow.clone();
  transformSceneFlow3d(flow3d, d_curr, d_next, MAX_DEPTH);
  FileOperation::writeSceneFlowDat(flow3d, frame_num);

  // 部分領域の抽出
  /*GraphBased gb(sigma, ap_filter, k, min_size, w_I, w_D, w_F,
    color_sigma, smooth_it);*/

  // 平滑化
  Mat c_smooth = c_curr.clone();
  Mat d_smooth = d_curr.clone();
  smoothing(c_smooth, d_smooth);

  c_curr.convertTo(c_curr, CV_32FC3);
  d_curr.convertTo(d_curr, CV_32FC1);

  GraphBased gb(k, min_size, w_I, w_D, w_F);
  SubRegion region = gb.segmentImage(c_smooth, d_smooth, flow3d);

  // 特徴量の計算の前処理（部分領域の境界近傍のフローを除去）
  Mat boundary;
  label.release();
  int region_num = preprocess(label, boundary, region);

  // 特徴量の計算
  Features f_vec = calcFeatures(label, frame_num, region_num,
    c_curr, d_curr, flow, boundary);

  // 境界の取得
  boundarys.push_back(boundary);

  char name[80];
  string buf;

  // 描画処理
  // 各フレームの部分領域を描画
  sprintf(name, "./out_subregion/subregion-0%004d.png", frame_num);
  buf = name;
  Mat draw = drawSubRegion(label);
//  Mat move = drawMovingSubRegion(label, f_vec, region, 10);
  //Mat opt_flow = drawOpticalFlow(c_curr, flow, boundary, 3, ap_flow);
  Mat average = drawSubRegionAverage(label, boundary, f_vec);
  Mat region_flow = drawSubRegionFeature(label, boundary, f_vec, 5, 10);
  imwrite(buf, draw);

  sprintf(name, "./out_subregion/subregionflow-0%004d.png", frame_num);
  buf = name;
  imwrite(buf, region_flow);

  sprintf(name, "./out_subregion/average-0%004d.png", frame_num);
  buf = name;
  imwrite(buf, average);

  sprintf(name, "./subregion/boundary-0%004d.png", frame_num);
  imwrite(name, drawBoundary(label, ap_boundary));

  imshow("subregion", draw);
  imshow("region feature", region_flow);
  waitKey(10);

  return f_vec;
}

//Features SubRegionExtraction::correctSubRegionExtract(Mat &label, const int frame_num,
//  const Mat &c_prev, const Mat &d_prev, const Mat &c_curr, const Mat &d_curr,
//  const Mat &c_next, const Mat &d_next, vector<Mat> &boundarys)
//{
//  // 過去フレームのフローを推定
//  Mat flow;
//  SceneFlow::sceneFlowFarneback(c_prev, d_prev, c_curr, d_curr, flow, scale,
//    levels, ap_flow, iterations, ap_flow / pow((float)(1 / scale), (float)(levels - 1)), 1,1);
//
//  // 部分領域の抽出
//  TemporalGraphBased gb(sigma, ap_filter, k, min_size, w_I, w_D, 
//    color_sigma, smooth_it);
//  gb.comulativeBoundary(boundarys);
//  //gb.useFlow(flow);
//  SubRegion region = gb.segmentCorrectImage(c_curr, d_curr, c_prev, d_prev, label);
//
//  // シーンフローの推定
//  //Mat flow;
//  SceneFlow::sceneFlowFarneback(c_curr, d_curr, c_next, d_next, flow, scale,
//    levels, ap_flow, iterations, ap_flow / pow((float)(1 / scale), (float)(levels - 1)), 1,1);
//
//  // 特徴量の計算の前処理（部分領域の境界近傍のフローを除去）
//  Mat boundary;
//  label.release();
//  int region_num = preprocess(label, boundary, region);
//
//  // 特徴量の計算
//  Features f_vec = calcFeatures(label, frame_num, region_num,
//    c_curr, d_curr, flow, boundary);
//
//  // 境界の取得
//  boundarys.push_back(boundary);
//
//  char name[80];
//  string buf;
//
//  // 描画処理
//  // 各フレームの部分領域を描画
//  sprintf(name, "./out_subregion/subregion-0%004d.png", frame_num);
//  buf = name;
//  Mat draw = drawSubRegion(label);
//  imwrite(buf, draw);
//  imshow("subregion", draw);
//  waitKey(10);
//
//  // シーンフローのファイル出力
//  FileOperation::writeSceneFlowDat(flow, frame_num);
//
//  return f_vec;
//}

/**
 * @brief 平滑化処理
 * @param[in, out] color 画像情報
 * @param[in, out] depth 深度情報
 */
void SubRegionExtraction::smoothing(Mat &color, Mat &depth)
{
  // チャンネル毎に平滑化
  Mat color32f, depth32f;
  color.convertTo(color32f, CV_32FC3);
  depth.convertTo(depth32f, CV_32FC1);

  vector<Mat> bgr;
  split(color32f, bgr);

  Mat smooth_r = bgr[2].clone();
  Mat smooth_g = bgr[1].clone();
  Mat smooth_b = bgr[0].clone();
  Mat smooth_d = depth32f.clone();

  if (sigma != 0.0) {
    // 平滑化を繰り返す
    for (int i = 0; i < smooth_it; i++) {
      // バイラテラルフィルタを利用
      if (color_sigma != 0.0) {
        Smooth::bilateralFiler(smooth_r, smooth_r, ap_filter, color_sigma, sigma);
        Smooth::bilateralFiler(smooth_g, smooth_g, ap_filter, color_sigma, sigma);
        Smooth::bilateralFiler(smooth_b, smooth_b, ap_filter, color_sigma, sigma);
        Smooth::bilateralFiler(smooth_d, smooth_d, ap_filter, color_sigma, sigma, 0);
      }
      // ガウシアンフィルタを利用
      else {
        Smooth::gaussianFilter(smooth_r, smooth_r, ap_filter, sigma);
        Smooth::gaussianFilter(smooth_g, smooth_g, ap_filter, sigma);
        Smooth::gaussianFilter(smooth_b, smooth_b, ap_filter, sigma);
        Smooth::gaussianFilter(smooth_d, smooth_d, ap_filter, sigma, 0);
      }
    }
  }

  vector<Mat> mvc;
  mvc.push_back(smooth_b);
  mvc.push_back(smooth_g);
  mvc.push_back(smooth_r);
  
  merge(mvc, color);
  depth = smooth_d;
}

/**
 * @brief 前処理（ラベルの取得と部分領域の境界を抽出）
 * @param[out] label   部分領域のラベル
 * @param[out] boundary  部分領域の境界
 * @param[in] region  部分領域
 * @return  部分領域の数
 */
int SubRegionExtraction::preprocess(Mat &label, Mat &boundary, SubRegion &region)
{
  int width = region.width;
  int height = region.height;

  label = Mat::ones(height, width, CV_32S) * -1;
  boundary = Mat::zeros(height, width, CV_8UC1);

  vector<int> tmp;
  for (int i = 0; i < width * height; i++) {
    tmp.push_back(-1);
  }

  int region_num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = region.find(y * width + x);

      if (tmp[comp] == -1) {
        tmp[comp] = region_num;
        region_num++;
      }

      // 部分領域のラベルを取得
      label.at<int>(Point(x, y)) = tmp[comp];

      for (int yy = y; yy < y + 2 && yy < height; yy++) {
        for (int xx = x; xx < x + 2 && xx < width; xx++) {
          int comp2 = region.find(yy * width + xx);
          if (comp != comp2) {
            // 部分領域の境界の描画
            line(boundary, Point(x, y), Point(x, y), 255);
          }
        }
      }
    }
  }

  return region_num;
}


/**
* @brief 部分領域の特徴量の計算
* @param[out] label  ラベル
* @param[in]  frame_num フレーム番号
* @param[in]  region_num 部分領域数
* @param[in]  color   画像情報
* @param[in]  depth   深度情報
* @param[in]  flow    運動情報（シーンフロー）
* @param[in]  boundary  部分領域の境界
* @return    部分領域の特徴量のリスト
*/
Features SubRegionExtraction::calcFeatures(Mat &label, const int frame_num,
  const int region_num, const Mat &color, const Mat &depth, const Mat &flow,
  const Mat &boundary)
{
  int width = color.cols;
  int height = color.rows;

  Features f(region_num);

  // シーンフローのヒストグラムの初期化
  vector<Mat> delxyz;
  split(flow, delxyz);

  Point3d min, max;
  minMaxLoc(delxyz[0], &min.x, &max.x);
  minMaxLoc(delxyz[1], &min.y, &max.y);
  minMaxLoc(delxyz[2], &min.z, &max.z);

  vector<vector<int>> histgram_x(region_num);
  vector<vector<int>> histgram_y(region_num);
  vector<vector<int>> histgram_z(region_num);
  for (int i = 0; i < region_num; i++) {
    histgram_x[i].resize(cvCeil(max.x) - cvFloor(min.x) + 1);
    histgram_y[i].resize(cvCeil(max.y) - cvFloor(min.y) + 1);
    histgram_z[i].resize(cvCeil(max.z) - cvFloor(min.z) + 1);
  }

  /////////////////////////////////////////////////////////////////////////////
  vector<vector<int>> median_x(region_num);
  vector<vector<int>> median_y(region_num);
  vector<vector<int>> median_z(region_num);
  /////////////////////////////////////////////////////////////////////////////

  // 部分領域ごとに特徴量の総数を計算
  vector<int> flow_size(region_num);
  vector<int> remove_size(region_num);
  bool boundary_flag;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int l = label.at<int>(Point(x, y));

      // RGB-D値の総数を計算
      Vec3f bgr = color.at<Vec3f>(Point(x, y));
      f[l].r += bgr[2];
      f[l].g += bgr[1];
      f[l].b += bgr[0];
      float d = depth.at<float>(Point(x, y));
      f[l].d += d;

      // 深度の欠損値の総数を計算
      if (d == 0) {
        remove_size[l]++;
      }

      // 領域サイズ，重心を計算
      f[l].size++;
      f[l].centroid.x += x;
      f[l].centroid.y += y;

      // 境界近傍のフローを除去
      boundary_flag = false;
      int rad = ap_flow / 2;
      for (int y2 = y - rad; y2 <= y + rad && y2 < height && 0 <= y2; y2++) {
        for (int x2 = x - rad; x2 <= x + rad && x2 < width && 0 <= x2; x2++) {
          if (boundary.at<uchar>(Point(x2, y2)) != 0) {
            boundary_flag = true;
          }
        }
      }

      if (boundary_flag) {
        continue;
      }

      // シーンフローのヒストグラムの作成
      histgram_x[l][cvRound(delxyz[0].at<float>(Point(x, y))) - cvFloor(min.x)]++;
      histgram_y[l][cvRound(delxyz[1].at<float>(Point(x, y))) - cvFloor(min.y)]++;
      histgram_z[l][cvRound(delxyz[2].at<float>(Point(x, y))) - cvFloor(min.z)]++;
      flow_size[l]++;

      // シーンフローの最頻値を計算するためのリストを作成
      median_x[l].push_back(delxyz[0].at<float>(Point(x, y)));
      median_y[l].push_back(delxyz[1].at<float>(Point(x, y)));
      median_z[l].push_back(delxyz[2].at<float>(Point(x, y)));
    }
  }

  // 部分領域ごとに特徴量の平均を計算
  for (int i = 0; i < region_num; i++) {
    if (f[i].size != 0) {
      f[i].num = frame_num;
      f[i].label = i;
      f[i].r /= f[i].size;
      f[i].g /= f[i].size;
      f[i].b /= f[i].size;
      if (f[i].size == remove_size[i]) {
        f[i].d = 0;
      }
      else {
        f[i].d /= (f[i].size - remove_size[i]);
      }
      f[i].centroid.x /= f[i].size;
      f[i].centroid.y /= f[i].size;

      // シーンフローのモードの計算
      if (flow_size[i] > 0) {
        f[i].flow2d.x = histgramMode(histgram_x[i], cvCeil(max.x), cvFloor(min.x), 1);
        f[i].flow2d.y = histgramMode(histgram_y[i], cvCeil(max.y), cvFloor(min.y), 1);
        f[i].flow2d.z = histgramMode(histgram_z[i], cvCeil(max.z), cvFloor(min.z), 1);

        ///////////////////////////////////////////////////////////////////////
        // シーンフローの中央値を計算
        // 各リストをソート
        sort(median_x[i].begin(), median_x[i].end());
        sort(median_y[i].begin(), median_y[i].end());
        sort(median_z[i].begin(), median_z[i].end());

        if ((flow_size[i] % 2) != 0) {
          f[i].flow2d.x = median_x[i][cvFloor(flow_size[i] / 2.0)];
          f[i].flow2d.y = median_y[i][cvFloor(flow_size[i] / 2.0)];
          f[i].flow2d.z = median_z[i][cvFloor(flow_size[i] / 2.0)];
        }
        else {
          f[i].flow2d.x = (median_x[i][(flow_size[i] / 2)] + 
            median_x[i][(flow_size[i] / 2) - 1]) / 2;
          f[i].flow2d.y = (median_y[i][(flow_size[i] / 2)] + 
            median_y[i][(flow_size[i] / 2) - 1]) / 2;
          f[i].flow2d.z = (median_z[i][(flow_size[i] / 2)] + 
            median_z[i][(flow_size[i] / 2) - 1]) / 2;
        }

        ///////////////////////////////////////////////////////////////////////

        // シーンフローの外れ値を除去
        if (abs(f[i].flow2d.z) > FLOW_THRESHOLD) {
          f[i].flow2d.z = 0;
        }
      }
      else {
        f[i].flow2d.x = 0;
        f[i].flow2d.y = 0;
        f[i].flow2d.z = 0;
      }

      // ノルムの計算
      f[i].norm2d = sqrt(square(f[i].flow2d.x) + square(f[i].flow2d.y)
        + square(f[i].flow2d.z));

      // シーンフローをmmに変換：手順１，仮想的な次のフレームの重心位置を計算
      f[i].flow3d = depth2World(f[i].centroid.x + f[i].flow2d.x,
        f[i].centroid.y + f[i].flow2d.y, luminance2Depth(f[i].d + f[i].flow2d.z),
        width, height);

      // 重心位置をmmに変換
      if (f[i].d != 0.0) {
        f[i].centroid3d = 
          depth2World(f[i].centroid.x, f[i].centroid.y, luminance2Depth(f[i].d), width, height);
      }
      else {
        f[i].centroid3d = 
          depth2World(f[i].centroid.x, f[i].centroid.y, 0, width, height);
      }

      // シーンフローをmmに変換：手順２
      f[i].flow3d = f[i].flow3d - f[i].centroid3d;

      // ノルムの計算
      f[i].norm3d = sqrt(square(f[i].flow3d.x) + square(f[i].flow3d.y)
        + square(f[i].flow3d.z));
    }
  }

  return f;
}

/**
* @brief モード（最頻値）の計算
* @param[in] histgram  ヒストグラム
* @param[in] max 要素の最大値
* @param[in] min 要素の最少値
* @param[in] interval  量子化間隔
* @return  ヒストグラムのモード
*/
float SubRegionExtraction::histgramMode(const vector<int> &histgram,
  const int max, const int min, const float interval)
{
  int max_elem = 0, argmax = 0;
  int range = (max - min) / interval;

  for (int i = 0; i <= range; i++) {
    if (max_elem < histgram.at(i)) {
      max_elem = histgram.at(i);
      argmax = i;
    }
  }

  return argmax * interval + min;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief オプティカルフローをdatファイルに出力
 * @param[in] flow  オプティカルフロー
 * @param[in] num   フレーム番号
 * @param[in] interval  フローの出力間隔
 */
void outOpticalFlowDat(const Mat &flow, //const Mat &boundary, const int aperture,
  const int num, const int interval)
{
  int width = flow.cols;
  int height = flow.rows;

  // 入力画像の輝度値をファイルに書き込み
  char name[80];
  sprintf(name, "./file/opticalflow%d.dat", num);
  FILE *fp = fopen(name, "w");

  //bool boundary_flag;
  //int rad = aperture / 2;
  for (int y = 0; y < height; y+= interval) {
    for (int x = 0; x < width; x+= interval) {
      Point3f velocity = flow.at<Point3f>(Point(x, y));

      // エッジ近傍のフローを除去
      //boundary_flag = false;
      //for (int y2 = y - rad; y2 <= y + rad && y2 < height && 0 <= y2; y2++) {
      //  for (int x2 = x - rad; x2 <= x + rad && x2 < width && 0 <= x2; x2++) {
      //    if (boundary.at<uchar>(Point(x2, y2)) != 0) {
      //      boundary_flag = true;
      //    }
      //  }
      //}

//      if (!boundary_flag) {
      float norm = sqrtf(square(velocity.x) + square(velocity.y) + square(velocity.z));
      if (abs(velocity.z) > 2 && abs(velocity.z) < 10)
        fprintf(fp, "%d, %d, %f, %f\n", x, y, velocity.x, velocity.y);
      //}
      //else {
      //  fprintf(fp, "%d, %d, %f, %f\n", x, y, 0, 0);
      //}
    }
//    fprintf(fp, "\n");
  }

  fclose(fp);
}

/**
 * @brief シーンフローをdatファイルに出力
 * @param[in] depth  深度情報
 * @param[in] flow  シーンフロー
 * @param[in] num   フレーム番号
 * @param[in] interval  フローの出力間隔
 */
void outSceneFlowDat(const Mat &depth, const Mat &flow, const int num, const int interval)
{
  int width = depth.cols;
  int height = depth.rows;

  // 入力画像の輝度値をファイルに書き込み
  char buf[80];
  sprintf(buf, "./file/sceneflow%d.dat", num);
  FILE *fp = fopen(buf, "w");

  for (int y = 0; y < height; y+= interval) {
    for (int x = 0; x < width; x+= interval) {
      Point3f velocity = flow.at<Point3f>(Point(x, y));
      uchar z = depth.at<uchar>(Point(x, y));
      float norm = sqrtf(square(velocity.x) + square(velocity.y) + square(velocity.z));
      if (abs(velocity.z) > 2 && abs(velocity.z) < 10)
        fprintf(fp, "%d %d %d %f %f %f\n", x, z, y, velocity.x, velocity.z, velocity.y);
    }
  }

  fclose(fp);
}
