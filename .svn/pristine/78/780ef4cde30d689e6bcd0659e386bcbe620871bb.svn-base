/*****************************************************************************/
/*! @addtogroup 
 *  @file   Misc.h
 *  @brief  構造体などをまとめたライブラリに関するファイル（ヘッダファイル）
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef MISC_H
#define MISC_H

/* Misc.h */
#include <vector>
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

//Debugモードの場合
#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)

#endif

using namespace cv;
using namespace std;

#ifndef SQUARE
/**
 * @brief 二乗和
 */
template <class T>
inline T square(const T &x) {
  return x * x;
};
#endif

/* マクロ定義 */
#define PI 3.14159265359  /*!<  円周率 */
#define THETA_X (57.0 / 2.0) / 180.0 * (float)PI  /*!< Kinectの水平視野角(仕様57°) */
#define THETA_Y (43.0 / 2.0) / 180.0 * (float)PI  /*!< 垂直視野角(仕様43°) */
#define MAX_DEPTH 3000.0  /*!<  深度情報の最大値(データの最大深度3000.0mmに設定しているため) */
#define MAX_LUM   255.0 /*!<  輝度の最大値(OpenCVの仕様) */
#define FLOW_THRESHOLD 20.0 /*!<  フローの外れ値を除去する閾値  */

/*! @struct TemporalVertex
 *  @brief  各フレームの部分領域を頂点とする構造体 
 */
struct TemporalVertex {
  int frame, label;

  TemporalVertex() {
    frame = 0;
    label = 0;
  }

  TemporalVertex(const int &f, const int &l) {
    frame = f;
    label = l;
  }

  friend bool operator==(const TemporalVertex &a, const TemporalVertex &b){
    return (a.frame == b.frame && a.label == b.label);
  }

  friend bool operator!=(const TemporalVertex &a, const TemporalVertex &b){
    return !(a.frame == b.frame && a.label == b.label);
  }

  friend bool operator < (const TemporalVertex &a, const TemporalVertex &b) {
    
  }
};

/*! @struct TemporalEdge
 *  @brief  対応付け用のエッジ構造体 
 */
struct TemporalEdge {
  float w;  // エッジの重み
  TemporalVertex p; /*!< ノード(前フレーム) */
  TemporalVertex n; /*!< ノード(次フレーム) */
  bool pflag;   /*!< 対応付けの方向(前のフレーム) */
  bool nflag;   /*!< 対応付けの方向(次のフレーム) */

  /**
   * @brief コンストラクタ
   */
  TemporalEdge() {
    pflag = false;
    nflag = false;
  };

  friend bool operator<(const TemporalEdge &a, const TemporalEdge &b) {
    return a.w < b.w;
  }

  friend bool operator==(const TemporalEdge &a, const TemporalEdge &b) {
    return (a.p == b.p && a.n == b.n);
  }
  friend bool operator!=(const TemporalEdge &a, const TemporalEdge &b) {
    return !(a.p == b.p && a.n == b.n);
  }
};

/*! @struct Feature
 *  @brief 部分領域の特徴量
 */
struct Feature {
  int num;  /*!< フレーム番号 */
  int label;  /*!< 部分領域のラベル */
  TemporalVertex root;  /*!< 領域系列の起点 */
  vector<TemporalEdge> prev;  /*!< 前フレームで対応付けした部分領域 */
  vector<TemporalEdge> next;  /*!< 次フレームで対応付けした部分領域 */
  int index;          /*!< 領域系列のラベル */
  Point2f centroid;   /*!< 重心 */
  Point3f centroid3d; /*!< 3次元の重心 */
  Point3f flow2d;     /*!< シーンフロー(画素＋輝度) */
  Point3f flow3d;     /*!< シーンフロー(mm) */
  float r, g, b;      /*!< 画像情報 */
  float d;            /*!< 深度情報 */
  float norm2d;       /*!< 2次元の移動量? */
  float norm3d;       /*!< 3次元の移動量 */
  float size;         /*!< 領域サイズ(画素数) */

  /**
   * @brief コンストラクタ
   */
  Feature() {
    num = -1;
    label = -1;
    index = -1;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid3d.x = 0.0;
    centroid3d.y = 0.0;
    centroid3d.z = 0.0;
    r = 0.0;
    g = 0.0;
    b = 0.0;
    d = 0.0;
    norm2d = 0.0;
    norm3d = 0.0;
    size = 0;
  }

  friend bool operator<(const Feature &f1, const Feature &f2) {
    return f1.norm2d < f2.norm2d;
  }
  friend bool operator>(const Feature &f1, const Feature &f2) {
    return f1.norm2d > f2.norm2d;
  }
  friend Feature operator+(const Feature &f1, const Feature &f2) {
    Feature f;
    f.num = f1.num;
    f.label = f1.label;
    f.centroid = f1.centroid + f2.centroid;
    f.centroid3d = f1.centroid3d + f2.centroid3d;
    f.flow2d = f1.flow2d + f2.flow2d;
    f.flow3d = f1.flow3d + f2.flow3d;
    f.r = f1.r + f2.r;
    f.g = f1.g + f2.g;
    f.b = f1.b + f2.b;
    f.d = f1.d + f2.d;
    f.norm2d = sqrt(square(f.flow2d.x) + square(f.flow2d.y)
        + square(f.flow2d.z));
    f.norm3d = sqrt(square(f.flow3d.x) + square(f.flow3d.y)
        + square(f.flow3d.z));
    f.size = f1.size + f2.size;
    return f;
  }

  friend Feature operator*(const Feature &f1, const Feature &f2) {
    Feature f;
    f.num = f1.num;
    f.label = f1.label;
    f.centroid = f1.centroid + f2.centroid;
    f.centroid3d = f1.centroid3d + f2.centroid3d;
    f.flow2d = f1.flow2d * f1.size + f2.flow2d * f2.size;
    f.flow3d = f1.flow3d * f1.size + f2.flow3d * f2.size;
    f.r = f1.r * f1.size + f2.r * f2.size;
    f.g = f1.g * f1.size + f2.g * f2.size;
    f.b = f1.b * f1.size + f2.b * f2.size;
    f.d = f1.d * f1.size + f2.d * f2.size;
    f.norm2d = sqrt(square(f.flow2d.x) + square(f.flow2d.y)
        + square(f.flow2d.z));
    f.norm3d = sqrt(square(f.flow3d.x) + square(f.flow3d.y)
        + square(f.flow3d.z));
    f.size = f1.size + f2.size;
    return f;
  }

  friend Feature operator/(Feature f1, int &n) {
    f1.centroid.x /= n;
    f1.centroid.y /= n;
    f1.centroid3d.x /= n;
    f1.centroid3d.y /= n;
    f1.centroid3d.z /= n;
    f1.flow2d.x /= n;
    f1.flow2d.y /= n;
    f1.flow2d.z /= n;
    f1.flow3d.x /= n;
    f1.flow3d.y /= n;
    f1.flow3d.z /= n;
    f1.r /= n;
    f1.g /= n;
    f1.b /= n;
    f1.d /= n;
    f1.norm2d = sqrt(square(f1.flow2d.x) + square(f1.flow2d.y)
        + square(f1.flow2d.z));
    f1.norm3d = sqrt(square(f1.flow3d.x) + square(f1.flow3d.y)
        + square(f1.flow3d.z));
    return f1;
  } 
};

/* 部分領域の特徴量の集合 */
typedef vector<Feature> Features; /*!< 部分領域の特徴量の集合 */

///* 部分領域の対応付け結果 */
//class TemporalSubRegion {
//private :
//  vector<vector<int>> n_ranks;
//  vector<vector<int>> p_ranks;
//  vector<vector<int>> frames;
//  vector<vector<int>> labels;
//  vector<vector<float>> norms;
//  
//  vector<vector<int>> real_frame;
//
//  int num;
//
//  int start;
//
//public :
//  TemporalSubRegion(int frame_num, vector<int> elements, vector<Features> &f_vec);
//  TemporalSubRegion(void){};
//  ~TemporalSubRegion(void);
//  int find(int frame, int label, int &f, int &l, int &r_next, int &r_prev);
//  void join(int f_prev, int l_prev, int f_next, int l_next);
//  int num_sets(void) const;
//  float getNorm(int frame, int label);
//
//  /////////////////////////////////////////////////////////////////////////////
//  void getLast(vector<int> &r_f, vector<int> &l) {
//    int last = labels.size() - 1;
//    r_f = real_frame.at(last);
//    l = labels.at(last);
//  }
//  void setStart(vector<int> &r_f, vector<int> &l) {
//    real_frame.at(0) = r_f;
//    labels.at(0) = l;
//  }
//  void setS(int start) {
//    this->start = start;
//  }
//  /////////////////////////////////////////////////////////////////////////////
//};

/*! @class TemporalRegion
 *  @brief 部分領域の対応付け結果を表すクラス 
 */
class TemporalRegion
{
private :
  vector<vector<TemporalVertex>> elts;  /*!< 要素(部分領域) */
  vector<vector<int>> ranks;            /*!< 要素のランク */
  int num;                              /*!< 要素数 */

public :
  TemporalRegion(int frame_num, vector<int> region_num);
  TemporalRegion(void){};
  ~TemporalRegion(void){};
  TemporalVertex find(TemporalVertex x);
  int findRank(TemporalVertex x);
  void join(TemporalVertex p, TemporalVertex n);
  void joinRoot(TemporalVertex root, TemporalVertex v);
//  void setRootLabel(TemporalVertex index, TemporalVertex root);
  int regionNum(void) const;
};

///* 部分領域の系列の特徴 */
//class FeatureSequence {
//private :
//  int frame;          // 部分領域の初期フレーム番号
//  int label;          // 部分領域のラベル
//  int length;         // 系列の長さ
//  Features features;  // 特徴の系列
////  vector<int> element;  // 特徴の有り無し判定
//  vector<vector<int>> labels;   // もとのラベル
//
//public :
//  FeatureSequence(int frame, int label, int length);
//  void assign(int num, Feature f);
//  void average(int num);
//  int getFrame(void) const;
//  int getLabel(void) const;
//  int getLength(void) const;
////  void setUp(void) { centroid.x /= element_num; centroid.y /= element_num; };
////  Point2f location(void) const { return centroid; };
//  Features getFeatures(void) const;
//  Feature getFeature(int num) const;
////  int getElement(int num) const {return element.at(num); };
//  vector<int> getLabels(int num) const;
//
//  void setLength(int length);
//};

/*! @class  RegionSequence
 *  @brief  領域系列のクラス構造体
 */
class RegionSequence
{
private :
  TemporalVertex label; /*!< 領域系列のラベル */
  int length;           /*!< 領域系列の長さ */
  float norm;           /*!< 領域系列の移動量 */
  vector<Features> features_vec;  /*!< 各フレームでの部分領域の特徴量 */

public :
  bool segment; /*!< 領域系列の分割判定フラグ */

  RegionSequence(TemporalVertex label, int max_length);
  void setLabel(TemporalVertex label);
  void setLength(int length);
  void setFeature(int frame_num, Feature f);
  TemporalVertex getLabel(void) const;
  int getLength(void) const;
  float getNorm(void) const;
  Features getFeatures(int frame_num) const;
  void calcNorm(void);
  void terminateSegment(void);

  RegionSequence operator + (RegionSequence seq);
  RegionSequence operator * (RegionSequence seq);
};

/* 領域系列の集合 */
typedef vector<RegionSequence> RegionSequences; /*!< 領域系列の集合 */

/* グローバル関数 */
float distanceCentroid(const Point2f &p, const Point2f &n, 
  const Point2f &flow = Point2f());
float distanceCentroid3d(const Point3f &p, const Point3f &n,
  const Point3f &flow = Point3f());
Point3f depth2World(const float pixel_x, const float pixel_y, const float depth,
  const int width, const int height);
void transformSceneFlow3d(Mat &flow, const Mat &d_curr,
  const Mat &d_next, const float max_depth);

// 深度を輝度値？から実測値に変換
float luminance2Depth(int d);

Features getFrameFeatures(RegionSequences sequences, int t);
bool featuresLabelAsc(const Feature &a, const Feature &b);

#endif