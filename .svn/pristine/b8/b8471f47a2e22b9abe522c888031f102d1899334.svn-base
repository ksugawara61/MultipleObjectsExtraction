/*****************************************************************************/
/*! @addtogroup 
 *  @file   Misc.cpp
 *  @brief  構造体などをまとめたライブラリに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "Misc.h"

//TemporalSubRegion::TemporalSubRegion(int frame_num, vector<int> elements,
//  vector<Features> &f_vec) 
//{
//  n_ranks.resize(frame_num);
//  p_ranks.resize(frame_num);
//  frames.resize(frame_num);
//  labels.resize(frame_num);
//  norms.resize(frame_num);
//  real_frame.resize(frame_num);
//
//  num = 0;
//
//  for (int i = 0; i < frame_num; i++) {
//    n_ranks.at(i).resize(elements.at(i));
//    p_ranks.at(i).resize(elements.at(i));
//    frames.at(i).resize(elements.at(i));
//    labels.at(i).resize(elements.at(i));
//    norms.at(i).resize(elements.at(i));
//    real_frame.at(i).resize(elements.at(i));
//  }
//
//  for (int y = 0; y < frame_num; y++) {
//    for (int x = 0; x < elements.at(y); x++) {
//      frames.at(y).at(x) = y;
//      labels.at(y).at(x) = x;
//      n_ranks.at(y).at(x) = 0;
//      p_ranks.at(y).at(x) = 0;
//      norms.at(y).at(x) = f_vec.at(y).at(x).norm3d;
//      real_frame.at(y).at(x) = f_vec.at(y).at(x).num;
//      num++;
//    }
//  }
//}
//
//TemporalSubRegion::~TemporalSubRegion(void)
//{
//}
//
//int TemporalSubRegion::find(int frame, int label, int &f, int &l, int &r_next, int &r_prev)
//{
//  int y = frame;
//  int x = label;
//  while(y != frames.at(y).at(x) || x != labels.at(y).at(x)) {    
//    int tmp = y;
//    y = frames.at(tmp).at(x);
//    x = labels.at(tmp).at(x);
//  }
//  frames.at(frame).at(label) = y;
//  labels.at(frame).at(label) = x;
//
//  real_frame.at(frame).at(label) = real_frame.at(y).at(x);
//
//  f = y;
//  l = x;
//  r_next = n_ranks.at(frame).at(label);
//  r_prev = p_ranks.at(frame).at(label);
//
//  return real_frame.at(y).at(x);
//}
//
//void TemporalSubRegion::join(int f_prev, int l_prev, int f_next, int l_next)
//{
//  frames.at(f_next).at(l_next) = f_prev;
//  labels.at(f_next).at(l_next) = l_prev;
//  n_ranks.at(f_next).at(l_next)++;
//  p_ranks.at(f_prev).at(l_prev)++;
//
//  real_frame.at(f_next).at(l_next) = real_frame.at(f_prev).at(l_prev);
//
//  float norm = max(norms.at(f_prev).at(l_prev), norms.at(f_next).at(l_next));
//  norms.at(f_prev).at(l_prev) = norm;
//  norms.at(f_next).at(l_next) = norm;
//
//  num--;
//}
//
//int TemporalSubRegion::num_sets(void) const { return num; };
//
//float TemporalSubRegion::getNorm(int frame, int label)
//{
//  int f = 0, l = 0, r_next = 0, r_prev = 0;
//  find(frame, label, f, l, r_next, r_prev);
//
//  return norms.at(f).at(l);
//}


/**
 * @brief コンストラクタ
 * @param[in] frame_num   フレームの総数
 * @param[in] region_num  各フレームでの部分領域の数
 */
TemporalRegion::TemporalRegion(int frame_num, vector<int> region_num)
{
  num = 0;

  // メモリの確保
  elts.resize(frame_num);
  ranks.resize(frame_num);
  for (int i = 0; i < frame_num; i++) {
    elts.at(i).resize(region_num.at(i));
    ranks.at(i).resize(region_num.at(i));
  }

  // 各要素の初期化
  for (int i = 0; i < frame_num; i++) {
    for (int j = 0; j < region_num.at(i); j++) {
      elts.at(i).at(j) = TemporalVertex(i, j);
      ranks.at(i).at(j) = 0;
      num++;
    }
  }
}

/**
 * @brief 部分領域の対応付け結果の探索
 * @param[in] x 対象の部分領域
 * @return  対応付けした部分領域
 */
TemporalVertex TemporalRegion::find(TemporalVertex x)
{
  TemporalVertex y = x;
  while(!(y == elts.at(y.frame).at(y.label))) {
    y = elts.at(y.frame).at(y.label);
  }

  elts.at(x.frame).at(x.label) = y;
  return y;
}

/**
 * @brief 部分領域の対応付け結果のランクの探索
 * @param[in] x 対象の部分領域
 * @return  対応付けのランク
 */
int TemporalRegion::findRank(TemporalVertex x)
{
  TemporalVertex y = x;
  while(y != elts.at(y.frame).at(y.label)) {
    y = elts.at(y.frame).at(y.label);
  }

  elts.at(x.frame).at(x.label) = y;
  return ranks.at(y.frame).at(y.label);
}

/**
 * @brief 部分領域の対応付け
 * @param[in] x 対応付けの対象の部分領域
 * @param[in] y 対応付けの対象の部分領域
 */
void TemporalRegion::join(TemporalVertex x, TemporalVertex y)
{
  if (ranks.at(x.frame).at(x.label) > ranks.at(y.frame).at(y.label)) {
    elts.at(y.frame).at(y.label) = x;
  }
  else {
    elts.at(x.frame).at(x.label) = y;
    if (ranks.at(x.frame).at(x.label) == ranks.at(y.frame).at(y.label)) {
      ranks.at(y.frame).at(y.label)++;
    }
  }
  num--;
}

/**
 * @brief 部分領域の起点を設定し対応付け
 * @param[in] root  対応付けの対象の部分領域（起点）
 * @param[in] v     対応付けの対象の部分領域
 */
void TemporalRegion::joinRoot(TemporalVertex root, TemporalVertex v)
{
  elts.at(v.frame).at(v.label) = root;
  ranks.at(v.frame).at(v.label)++;
  ranks.at(root.frame).at(root.label)++;
  num--;
}

//void TemporalRegion::setRootLabel(TemporalVertex index, TemporalVertex root)
//{
//  elts.at(index.frame).at(index.label) = root;
//}

/**
 * @brief 領域系列数を返す（移動しない領域系列を含む）
 */
int TemporalRegion::regionNum() const { return num; }



///* 部分領域の系列の特徴 */
//FeatureSequence::FeatureSequence(int frame, int label, int length)
//{
//  this->frame = frame;
//  this->label = label;
//  this->length = length;
//  features.resize(length);
//  labels.resize(length);
//  //    element.resize(length);
//}
//
//void FeatureSequence::assign(int num, Feature f) {
//  labels.at(num).push_back(f.label);
//  features.at(num) = f + features.at(num);
//  //    element.at(num)++;
//};
//
//void FeatureSequence::average(int num) {
//  int n = labels.at(num).size();
//  if (n != 0) {
//    features.at(num) = features.at(num) / n;
//  }
//};
//
//int FeatureSequence::getFrame(void) const { return frame; }
//int FeatureSequence::getLabel(void) const { return label; }
//int FeatureSequence::getLength(void) const { return length; }
////  void setUp(void) { centroid.x /= element_num; centroid.y /= element_num; };
////  Point2f location(void) const { return centroid; };
//Features FeatureSequence::getFeatures(void) const { return features; }
//Feature FeatureSequence::getFeature(int num) const { return features.at(num); }
////  int getElement(int num) const {return element.at(num); };
//vector<int> FeatureSequence::getLabels(int num) const { return labels.at(num); }
//
//// 部分領域系列の長さを格納
//void FeatureSequence::setLength(int length)
//{
//  this->length = length;
//}


// 領域系列の特徴量
/**
 * @brief コンストラクタ
 * @param[in] label       領域系列のラベル
 * @param[in] max_length  処理の最大フレーム数
 */
RegionSequence::RegionSequence(TemporalVertex label, int max_length)
{
  this->label = label;
  this->length = 0;
  this->segment = true;
  this->norm = 0.0;
  features_vec.resize(max_length);
}

/**
 * @brief 領域系列のラベルの設定
 * @param[in] label 領域系列のラベル
 */
void RegionSequence::setLabel(TemporalVertex label) { this->label = label; }

/**
 * @brief 領域系列のフレーム数の設定
 * @param[in] length 領域系列のフレーム数
 */
void RegionSequence::setLength(int length) { this->length = length; }

/**
 * @brief 領域系列の特徴量の追加
 * @param[in] frame_num 対象のフレーム番号
 * @param[in] f         領域系列に追加するフレーム番号の部分領域
 */
void RegionSequence::setFeature(int frame_num, Feature f)
{
  Features &pf = features_vec.at(frame_num);

  if (pf.size() == 0) {
    length++;
  }

  pf.push_back(f);
}

/**
 * @brief 領域系列のラベルの取得
 * @return  領域系列のラベル
 */
TemporalVertex RegionSequence::getLabel(void) const { return label; }

/**
 * @brief 領域系列のフレーム数の取得
 * @return  領域系列のフレーム数
 */
int RegionSequence::getLength(void) const { return length; }

/**
 * @brief 領域系列の移動量の取得
 * @return  領域系列の移動量
 */
float RegionSequence::getNorm(void) const { return norm; }

/**
 * @brief 領域系列の特徴量の取得
 * @param[in] frame_num フレーム番号
 * @return  領域系列のframe_numの特徴量（部分領域の配列）
 */
Features RegionSequence::getFeatures(int frame_num) const
{ 
  return features_vec.at(frame_num); 
}

/**
 * @brief 領域系列の移動量の平均を計算（部分領域のシーンフローの加重平均）
 */
void RegionSequence::calcNorm(void)
{
  int max_length = features_vec.size();
  for (int i = 0; i < max_length; i++) {
    Features &pf = features_vec.at(i);
    
    // 領域の総画素数を計算
    int size = 0;
    for (int j = 0; j < pf.size(); j++) {
      size += pf.at(j).size;
    }

    // シーンフローの平均を計算
    for (int j = 0; j < pf.size(); j++) {
      norm += (pf.at(j).size * pf.at(j).norm3d) / size;
    }
  }

  // 領域系列のシーンフローの平均を計算
  norm /= length;
}

/**
 * @brief 領域系列の分割を終了
 */
void RegionSequence::terminateSegment(void)
{
  segment = false;
}

/**
 * @brief 領域系列の和
 */
RegionSequence RegionSequence::operator + (RegionSequence seq)
{
  int max_length = this->features_vec.size();
  RegionSequence tmp_seq = *this;
  TemporalVertex root = tmp_seq.label;
  for (int i = 0; i < max_length; i++) {
    Features f1 = tmp_seq.getFeatures(i);
    Features f2 = seq.getFeatures(i);
    for (int j = 0; j < f2.size(); j++) {
      bool flag = false;
      for (int k = 0; k < f1.size(); k++) {
        if (f1.at(k).num == f2.at(j).num && f1.at(k).label == f2.at(j).label) {
          flag = true;
        }
      }
      if (!flag) {
        f2.at(j).root = root;
        tmp_seq.setFeature(i, f2.at(j));
      }
    }
  }

  return tmp_seq;
}

/**
 * @brief 領域系列の積
 */
RegionSequence RegionSequence::operator * (RegionSequence seq)
{
  int max_length = this->features_vec.size();
  RegionSequence tmp_seq(this->label, max_length);
  for (int i = 0; i < max_length; i++) {
    Features f1 = tmp_seq.getFeatures(i);
    Features f2 = seq.getFeatures(i);
    for (int j = 0; j < f2.size(); j++) {
      for (int k = 0; k < f1.size(); k++) {
        if (f1.at(k).num == f2.at(j).num && f1.at(k).label == f2.at(j).label) {
          tmp_seq.setFeature(i, f1.at(k));
        }
      }
    }
  }

  return tmp_seq;
}

// グローバル関数
/**
 * @brief （重心）距離の計算
 * @param[in] p 前フレームの重心の画素
 * @param[in] n 後フレームの重心の画素
 * @param[in] flow  前フレームのオプティカルフロー
 * @return （重心）距離
 */
float distanceCentroid(const Point2f &p, const Point2f &n, 
  const Point2f &flow)
{
  float dist_centroid = sqrtf((square((p.x + flow.x) - n.x)) 
    + (square((p.y + flow.y) - n.y)));
  return dist_centroid;
};

/**
 * @brief 3次元の（重心）距離の計算
 * @param[in] p 前フレームの重心の3次元位置
 * @param[in] n 後フレームの重心の3次元位置
 * @param[in] flow  前フレームのシーンフロー
 * @return 3次元の（重心）距離
 */
float distanceCentroid3d(const Point3f &p, const Point3f &n,
  const Point3f &flow)
{
  return sqrtf((square((p.x + flow.x) - n.x))
    + (square((p.y + flow.y) - n.y)) + (square((p.z + flow.z) - n.z)));
};

/**
* @brief pixel→mm変換
* @param[in]  pixel_x xの画素位置
* @param[in]  pixel_y yの画素位置
* @param[in]  depth   画素の深度の値
* @param[in]  width   横幅
* @param[in]  height  縦幅
* @return  変換後の値
*/
Point3f depth2World(const float pixel_x, const float pixel_y,
  const float depth, const int width, const int height)
{
  float	W;  // 深度がdepthのときの最大水平視野(mm)
  float	H;  // 深度がdepthのときの最大垂直視野(mm)
  Point	P;  // 画像から撮れる画像の最大pixel
  Point	p;  // 画像中心から座標点までのpixel
  float	w;  // 中心から座標点までの水平視野(mm)
  float	h;  // 中心から座標点までの垂直視野(mm)
  Point3f	pt;  // 実世界での三次元座標

  // Kinectから座標点までの幅(mm)を求める式
  W = (float)tan(THETA_X) * depth;
  P.x = width / 2;
  p.x = pixel_x - P.x;
  w = p.x * W / P.x;

  // Kinectから座標点までの高さ(mm)を求める式
  H = (float)tan(THETA_Y) * depth;
  P.y = height / 2;
  p.y = P.y - pixel_y;
  h = p.y * H / P.y;

  // 値を代入
  pt.x = w;
  pt.y = h;
  pt.z = depth;

  return pt;
}

/**
* @brief シーンフローをpixel→mmへ変換
* @param[in,out] flow 変換するフローの行列（画素＋輝度→mm）
* @param[in]  d_curr  現在の深度画像
* @param[in]  d_next  次の深度画像
* @param[in]  max_depth 距離の最大計測値（mm）
*/
void transformSceneFlow3d(Mat &flow, const Mat &d_curr,
  const Mat &d_next, const float max_depth)
{
  int width = flow.cols;
  int height = flow.rows;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Point3f &delta = flow.at<Point3f>(y, x);

      // 深度を画素値(8bit)からmmへ変換
      float depth = (float)d_curr.at<uchar>(y, x) / 255.0 * max_depth;
      Point3f pt_prev = depth2World(x, y, depth, width, height);

      Point trans(x + (int)delta.x, y + (int)delta.y);
      if (trans.x < 0 || trans.x >= width || trans.y < 0 || trans.y >= height) {
        delta = Point3f(0, 0, 0);
        continue;
      }
      depth = (float)d_next.at<uchar>(trans) / 255.0 * max_depth;
      Point3f pt_next = depth2World(x, y, depth, width, height);

      // シーンフローの更新
      delta.x = pt_next.x - pt_prev.x;
      delta.y = pt_next.y - pt_prev.y;
      delta.z = pt_next.z - pt_prev.z;
    }
  }
}

/**
 * @brief 深度の変換（輝度→mm）
 * @param[in] d 深度（輝度）
 * @return  深度（mm）
 */
float luminance2Depth(int d)
{
  return d / MAX_LUM * MAX_DEPTH;
}

/**
 * @brief フレームtでの領域系列の集合に属する部分領域の取得
 * @param[in] sequences 領域系列の配列
 * @param[in] t フレーム番号
 * @return フレームtでの領域系列の集合に属する部分領域
 */
Features getFrameFeatures(RegionSequences sequences, int t)
{
  Features f;
  for (int i = 0; i < sequences.size(); i++) {
    Features &pf = sequences.at(i).getFeatures(t);
    for (int j = 0; j < pf.size(); j++) {
      pf.at(j).index = i; // 領域系列番号を格納
      f.push_back(pf.at(j));
    }
//    f.insert(f.end(), pf.begin(), pf.end());
  }

  return f;
}

/**
 * @brief 部分領域をラベルの昇順でソートする関数
 */
bool featuresLabelAsc(const Feature &a, const Feature &b)
{
  return a.num == b.num ? a.label < b.label : a.num < b.num;
}