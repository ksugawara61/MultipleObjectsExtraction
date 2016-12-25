/*****************************************************************************/
/*! @addtogroup 
 *  @file   MotionVector.cpp
 *  @brief  運動情報の処理に関するライブラリ
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "MotionVector.h"

/**
 * @brief ブロックマッチング法によるオプティカルフローの計算
 * @param[in] prev  1番目の画像
 * @param[in] next  2番目の画像
 * @param[in,out]  flow  オプティカルフローの計算結果
 * @param[in] block 比較対象となる基本ブロックのサイズ
 * @param[in] shift ブロック座標の増加量
 * @param[in] max_range 走査対象のブロックの近傍領域
 * @param[in] use_previous 前回のフローを利用
 */
void OpticalFlow::opticalFlowBM(Mat prev, Mat next, Mat &flow, 
    Size block, Size shift, Size max_range, int use_previous)
{
  if (prev.channels() == 3) {
    cvtColor(prev, prev, CV_BGR2GRAY);
  }
  if (next.channels() == 3) {
    cvtColor(next, next, CV_BGR2GRAY);
  }

  IplImage src_img1 = prev;
  IplImage src_img2 = next;
  CvMat *velx, *vely;

  //int rows = (&src_img1)->height;
  //int cols = (&src_img1)->width;
  int rows = int (floor (double (src_img1.height - block.height + shift.height) / shift.height));
  int cols = int (floor (double (src_img1.width - block.width + shift.width) / shift.width));
  velx = cvCreateMat (rows, cols, CV_32FC1);
  vely = cvCreateMat (rows, cols, CV_32FC1);
  cvSetZero (velx);
  cvSetZero (vely);

  // オプティカルフローの計算(BM法)
  cvCalcOpticalFlowBM(&src_img1, &src_img2, block, shift, max_range, use_previous, velx, vely);
  
  std::vector<Mat> delta;
  delta.push_back(velx);
  delta.push_back(vely);

  cv::merge(delta, flow);
}

/**
 * @brief Horn & Schunckアルゴリズムによるオプティカルフローの計算
 * @param[in] prev  1番目の画像
 * @param[in] next  2番目の画像
 * @param[in,out]  flow  オプティカルフローの計算結果
 * @param[in] use_previous  前回のフローを利用
 * @param[in] lambda  ラグランジュ乗数
 * @param[in] criteria  アルゴリズムの終了条件
 */
void OpticalFlow::opticalFlowHS(Mat prev, Mat next, Mat &flow, int use_previous, 
  double lambda, CvTermCriteria criteria)
{
  if (prev.channels() == 3) {
    cvtColor(prev, prev, CV_BGR2GRAY);
  }
  if (next.channels() == 3) {
    cvtColor(next, next, CV_BGR2GRAY);
  }

  IplImage src_img1 = prev;
  IplImage src_img2 = next;
  CvMat *velx, *vely;

  int rows = src_img1.height;
  int cols = src_img1.width;
  velx = cvCreateMat (rows, cols, CV_32FC1);
  vely = cvCreateMat (rows, cols, CV_32FC1);
  cvSetZero (velx);
  cvSetZero (vely);

  // オプティカルフローの計算(HS法)
  cvCalcOpticalFlowHS(&src_img1, &src_img2, use_previous, velx, vely, lambda, criteria);

  std::vector<Mat> delta;
  delta.push_back(velx);
  delta.push_back(vely);

  cv::merge(delta, flow);
}

/**
 * @brief Lucas & Kanadeアルゴリズムによるオプティカルフローの計算
 * @param[in] prev  1番目の画像
 * @param[in] next  2番目の画像
 * @param[out]  flow  オプティカルフローの計算結果
 * @param[in] win_size  アルゴリズムに用いるフロー計算の窓サイズ
 */
void OpticalFlow::opticalFlowLK(Mat prev, Mat next, Mat &flow, Size win_size)
{
  if (prev.channels() == 3) {
    cvtColor(prev, prev, CV_BGR2GRAY);
  }
  if (next.channels() == 3) {
    cvtColor(next, next, CV_BGR2GRAY);
  }

  IplImage src_img1 = prev;
  IplImage src_img2 = next;
  CvMat *velx, *vely;

  int rows = src_img1.height;
  int cols = src_img1.width;
  velx = cvCreateMat (rows, cols, CV_32FC1);
  vely = cvCreateMat (rows, cols, CV_32FC1);
  cvSetZero (velx);
  cvSetZero (vely);

  // オプティカルフローの計算(LK法)
  cvCalcOpticalFlowLK(&src_img1, &src_img2, win_size, velx, vely);

  std::vector<Mat> delta;
  delta.push_back(velx);
  delta.push_back(vely);

  cv::merge(delta, flow);
}

/**
 * @brief Lucas & Kanadeアルゴリズム+画像ピラミッドに基づくオプティカルフローの計算
 */
void OpticalFlow::opticalFlowPyrLK(Mat prev, Mat next, Mat &flow, 
  Size win_size, int levels, TermCriteria criteria, double deriv_lambda, int flags)
{
  if (prev.channels() == 3) {
    cvtColor(prev, prev, CV_BGR2GRAY);
  }
  if (next.channels() == 3) {
    cvtColor(next, next, CV_BGR2GRAY);
  }

  int max_corners = 1000;
  double quality_level = 0.05;
  double min_distance = 5.0;

  std::vector<Point2f> prev_pts(max_corners);
  std::vector<Point2f> next_pts(max_corners);
  Mat status, err;

  goodFeaturesToTrack(prev, prev_pts, max_corners, quality_level, min_distance);
  goodFeaturesToTrack(next, next_pts, max_corners, quality_level, min_distance);

  /*Point2f center = Point(prev.cols / 2., prev.rows / 2.);
  for (int i = 0; i < win_size.width; i++) {
    for (int j = 0; j < win_size.width; j++) {
      Point2f p(i * float(prev.cols)/(win_size.width - 1), 
        j * float(prev.rows)/(win_size.height - 1));
      prev_pts.push_back((p - center) * 0.9f + center);
    }
  }*/

  // Lucas & Kanadeアルゴリズム+画像ピラミッドに基づくオプティカルフローの計算
  calcOpticalFlowPyrLK(prev, next, prev_pts, next_pts, 
    status, err, win_size, levels, criteria, flags);
  

  vector<Point2f>::const_iterator p = prev_pts.begin();
  vector<Point2f>::const_iterator n = next_pts.begin();
  flow = Mat::zeros(prev.size(), CV_32FC2);

  for (; n != next_pts.end(); n++, p++) {
    Point2f &pflow = flow.at<Point2f>(*p);
    pflow = *n - *p;
  }
}

/**
 * @brief Farnebackのアルゴリズム
 */
void OpticalFlow::opticalFlowFarneback(Mat prev, Mat next, Mat &flow,
  double pyrscale, int levels, int winsize, int iterations, 
  int polyN, double polysigma, int flags)
{
  if (prev.channels() == 3) {
    cvtColor(prev, prev, CV_BGR2GRAY);
  }
  if (next.channels() == 3) {
    cvtColor(next, next, CV_BGR2GRAY);
  }

  // オプティカルフローの計算（Farneback）
  calcOpticalFlowFarneback(prev, next, flow, pyrscale, levels, winsize, iterations, 
    polyN, polysigma, flags);
}

/**
 * @brief 
 */
void OpticalFlow::opticalFlowDualTVL1(Mat prev, Mat next, Mat &flow)
{
  Mat velx, vely;

  Ptr<DenseOpticalFlowExt> opticalFlow = superres::createOptFlow_DualTVL1();
  opticalFlow->calc(prev, next, velx, vely);

  std::vector<Mat> delta;
  delta.push_back(velx);
  delta.push_back(vely);

  cv::merge(delta, flow);
}

/**
 * @brief SimpleFlowのアルゴリズム
 */
void OpticalFlow::opticalFlowSimple(Mat prev, Mat next, Mat &flow)
{
  Mat velx, vely;

  Ptr<DenseOpticalFlowExt> opticalFlow = superres::createOptFlow_Simple();
  opticalFlow->calc(prev, next, velx, vely);
  
  std::vector<Mat> delta;
  delta.push_back(velx);
  delta.push_back(vely);

  cv::merge(delta, flow);
}

/* シーンフローの推定 */
/**
 * @brief ブロックマッチング法＋深度情報
 */
void SceneFlow::sceneFlowBM(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow, 
  Size block, Size shift, Size max_range, int use_previous)
{
  OpticalFlow::opticalFlowBM(c_prev, c_next, flow, block, shift, max_range, use_previous);

  calcVelZ(d_prev, d_next, flow);
}

/**
 * @brief Horn & Shunkのアルゴリズム＋深度情報
 */
void SceneFlow::sceneFlowHS(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow,
  int use_previous, double lambda, CvTermCriteria criteria)
{
  OpticalFlow::opticalFlowHS(c_prev, c_next, flow, use_previous, lambda, criteria);

  calcVelZ(d_prev, d_next, flow);
}

/**
 * @brief Lucas & Kanadeのアルゴリズム＋深度情報
 */
void SceneFlow::sceneFlowLK(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, 
  Mat &flow, Size win_size)
{
  OpticalFlow::opticalFlowLK(c_prev, c_next, flow, win_size);

  calcVelZ(d_prev, d_next, flow);
}

/**
 * @brief Lucas & Kanadeアルゴリズム+画像ピラミッドに基づくアルゴリズム＋深度情報
 */
void SceneFlow::sceneFlowPyrLK(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, 
  Mat &flow, Size win_size, int levels, TermCriteria criteria,
  double deriv_lambda, int flags)
{
  OpticalFlow::opticalFlowPyrLK(c_prev, c_next, flow, win_size, levels,
    criteria, deriv_lambda, flags);

  calcVelZ(d_prev, d_next, flow);
}


/**
 * @brief Farnebackのアルゴリズム＋深度情報
 */
void SceneFlow::sceneFlowFarneback(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, 
  Mat &flow, double pyrscale, int levels, int winsize, int iterations, 
  int polyN, double polysigma, int flags)
{
  OpticalFlow::opticalFlowFarneback(c_prev, c_next, flow, pyrscale, levels,
    winsize, iterations, polyN, polysigma, flags);

  calcVelZ(d_prev, d_next, flow);
}

/**
 * @brief DualTVL1のアルゴリズム＋深度情報
 */
void SceneFlow::sceneFlowDualTVL1(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow)
{
  OpticalFlow::opticalFlowDualTVL1(c_prev, c_next, flow);

  calcVelZ(d_prev, d_next, flow);
}

/**
 *　@brief SimpleFlowのアルゴリズム＋深度情報
 */
void SceneFlow::sceneFlowSimple(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow)
{
  OpticalFlow::opticalFlowSimple(c_prev, c_next, flow);

  calcVelZ(d_prev, d_next, flow);
}

/**
 * @brief オプティカルフローの計算結果を基にシーンフローを計算
 * @param[in] d_prev  1番目のフレームの深度情報
 * @param[in] d_next  2番目のフレームの深度情報
 * @param[in,out] flow  シーンフローの計算結果
 */
void SceneFlow::calcVelZ(const Mat &d_prev, const Mat &d_next, Mat &flow)
{
  int width = flow.cols;
  int height = flow.rows;
  Mat velz = Mat::zeros(height, width, CV_32FC1);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Point2f velxy = flow.at<Point2f>(Point(x, y));
      if (x + velxy.x >= 0 && x + velxy.x < width && y + velxy.y >= 0 && y + velxy.y < height) {
        velz.at<float>(Point(x, y)) = d_next.at<uchar>(Point(x + (int)velxy.x, y + (int)velxy.y))
          - d_prev.at<uchar>(Point(x, y));
      }
      else {
        velz.at<float>(Point(x, y)) = 0.0;
      }
    }
  }

  vector<Mat> velocity;
  split(flow, velocity);
  velocity.push_back(velz);

  cv::merge(velocity, flow);
}