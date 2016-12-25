/*****************************************************************************/
/*! @addtogroup 
 *  @file   TemporalGraphBased.h
 *  @brief  時間方向に安定した部分領域への分割（抽出）処理に関するファイル（ヘッダファイル）
 *          ※ビルド時エラー発生
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef TEMPORALGRAPHBASED_H
#define TEMPORALGRAPHBASED_H

#include "../GraphBased/GraphBased.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/GraphBasedd.lib")
#else
#pragma comment(lib, "../x64/Release/GraphBased.lib")
#endif

static const float TC = 0.632;  // 時定数

/*! @class  ConfList
 *  @brief  
 */
class ConfList
{
public:
  ConfList() {};
  void initialize(int tag, float conf) {
    tag_list.push_back(tag);
    conf_list.push_back(conf);
  };
  void integrate(ConfList &list) {
    bool flag;
    for (int i = 0; i < list.tag_list.size(); i++) {
      flag = false;
      for (int j = 0; j < tag_list.size(); j++) {
        if (list.tag_list.at(i) == tag_list.at(j)) {
          conf_list.at(j) += list.conf_list.at(i);
          flag = true;
          break;
        }
      }

      if (!flag) {
        tag_list.push_back(list.tag_list.at(i));
        conf_list.push_back(list.conf_list.at(i));
      }
    }

    list.tag_list = tag_list;
    list.conf_list = conf_list;
  };
  int argmaxConf(void) {
    float max = 0;
    int argmax = -1;
    for (int i = 0; i < tag_list.size(); i++) {
      if (conf_list.at(i) > max) {
        argmax = tag_list.at(i);
        max = conf_list.at(i);
      }
    }

    return argmax;
  };
  
private:
  vector<int> tag_list;
  vector<float> conf_list;
};

/*! @class  TemporalGraphBased
 *  @brief  時間方向に安定した部分領域への分割を行うクラス
 *
 *  森 他, "時間方向に安定したSuperpixelの算出手法," 信学論 (D),
 *  Vol.J91-D, No.11, pp.2696-2708 (2008).の論文を基に実装
 */
class TemporalGraphBased : public GraphBased
{
public:
  TemporalGraphBased(float sigma = 0.5, int aperture = 3, float k = 500,
    int min_size = 20, float w_I = 1.0, float w_D = 3.0, float w_F = 0.1,
    float color_sigma = 0.0, int iteration = 1, 
    float rs = 1.0, float rd = 1, float alpha = 1.0, 
    float beta = 1.0, float gamma = 1.0);
  ~TemporalGraphBased(void);
  SubRegion segmentCorrectImage(Mat c_curr, Mat d_curr, Mat c_past,
    Mat d_past, Mat tag);

  void comulativeBoundary(const vector<Mat> &boundarys);
  void useFlow(const Mat &flow);

private:
  SubRegion segmentCorrectGraph(int width, int height, vector<edge> edges,
    const Mat &gray_curr, const Mat &gray_past, const Mat &tag);

  // 時間方向へ安定化
  float conf(const Mat &gray_curr, const Mat &gray_past, Point p);
  float diffTemporal(const Mat &gray_curr, const Mat &gray_past, Point p);
  float aveDiffTemporal(const Mat &gray_curr, const Mat &gray_past);

  float m_past;
  float rs;
  float rd;
  float gamma;
  float alpha;
  float beta;

  Mat comboundary;
  Mat flow;
};

#endif
