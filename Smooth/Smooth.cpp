/*****************************************************************************/
/*! @addtogroup 
 *  @file   Smooth.cpp
 *  @brief  画像・深度情報の平滑化ライブラリに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "Smooth.h"

/**
 * @brief Kinectの深度の欠損値を除去するためのメディアンフィルタ
 * @param[in]   src 入力画像
 * @param[out]  dst 出力画像
 * @param[in]   aperture  フィルタの窓サイズ
 * @param[in]   outliner  平滑化の計算に含めない外れ値の閾値
 */
void medianFilterForKinect(const Mat src, Mat &dst, int aperture, uchar outliner)
{
  int width = src.cols;
  int height = src.rows;
  dst = src.clone();

  // メディアンフィルタにより，深度の欠損値を除去
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      
      // 注目画素の平滑化
      uchar intensity = src.at<uchar>(y, x);

      if (intensity <= outliner) {
        vector<uchar> value(aperture * aperture);
        for (int fy = 0; fy < aperture; fy++) {
          for (int fx = 0; fx < aperture; fx++) {
            int index = fy * aperture + fx;
            int ix = x + fx - (aperture - 1) / 2;
            int iy = y + fy - (aperture - 1) / 2;
            ix = max(ix, 0);
            ix = min(ix, width - 1);
            iy = max(iy, 0);
            iy = min(iy, height - 1);

            value[y * aperture + x] = src.at<uchar>(iy, ix);
          }
        }

        // 輝度値をソート
        sort(value.begin(), value.end());

        if (value.size() / 2 != 0) {
          dst.at<uchar>(y, x) = value[cvFloor(value.size() / 2.0)];
        }
        else {
          dst.at<uchar>(y, x) = (value[value.size() / 2] +
            value[value.size() / 2 - 1]) / 2;
        }
      }
    }
  }
}

/**
 * @brief ガウシアンフィルタ
 * @param[in]   src 入力画像 32FC1
 * @param[out]  dst 出力画像 32FC1
 * @param[in]   aperture  フィルタの窓サイズ
 * @param[in]   sigma     ガウス分布の標準偏差
 */
void Smooth::gaussianFilter(const Mat src, Mat &dst, int aperture, float sigma)
{
  int width = src.cols;
  int height = src.rows;
  dst = Mat::zeros(height, width, CV_32FC1);

  // ガウシアンカーネルの生成
  if (aperture <= 0 && sigma > 0) {
    aperture = cvRound(sigma * 3.0 * 2 + 1)|1;
  }
  sigma = sigma > 0 ? sigma : 0.3 * ((aperture - 1.0) / 2.0 - 1.0) + 0.8;
  vector<float> kernel(aperture * aperture);
  float sum = 0.0;
  for (int y = 0; y < aperture; y++) {
    for (int x = 0; x < aperture; x++) {
      int index = y * aperture + x;
      float xx = x - (aperture - 1) / 2;
      float yy = y - (aperture - 1) / 2;
      kernel[index] = exp(- (square(xx) + square(yy)) / (2.0 * square(sigma)));
      sum += kernel[index];
    }
  }

  //// 正規化
  //for (int y = 0; y < aperture; y++) {
  //  for (int x = 0; x < aperture; x++) {
  //    int index = y * aperture + x;
  //    kernel[index] /= sum;
  //  }
  //}

  // ガウシアンフィルタによる平滑化
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      
      // 注目画素の平滑化
      float tmp = 0.0;
      for (int fy = 0; fy < aperture; fy++) {
        for (int fx = 0; fx < aperture; fx++) {
          int index = fy * aperture + fx;
          int ix = x + fx - (aperture - 1) / 2;
          int iy = y + fy - (aperture - 1) / 2;
          ix = max(ix, 0);
          ix = min(ix, width - 1);
          iy = max(iy, 0);
          iy = min(iy, height - 1);

          tmp += src.at<float>(Point(ix, iy)) * (kernel[index] / sum);
        }
      }
      dst.at<float>(Point(x, y)) = tmp;
    }
  }
}

/**
 * @brief 深度画像用ガウシアンフィルタ（オーバーロード）
 * @param[in]   src 入力画像 32FC1
 * @param[out]  dst 出力画像 32FC1
 * @param[in]   aperture  フィルタの窓サイズ
 * @param[in]   sigma     ガウス分布の標準偏差
 * @param[in]   outlier   平滑化の計算に含めない外れ値の閾値
 */
void Smooth::gaussianFilter(const Mat src, Mat &dst, 
  int aperture, float sigma, float outlier)
{
  int width = src.cols;
  int height = src.rows;
  dst = Mat::zeros(height, width, CV_32FC1);

  // ガウシアンカーネルの生成
  if (aperture <= 0 && sigma > 0) {
    aperture = cvRound(sigma * 3.0 * 2 + 1)|1;
  }
  sigma = sigma > 0 ? sigma : 0.3 * ((aperture - 1.0) / 2.0 - 1.0) + 0.8;
  vector<float> kernel(aperture * aperture);
  for (int y = 0; y < aperture; y++) {
    for (int x = 0; x < aperture; x++) {
      int index = y * aperture + x;
      float xx = x - (aperture - 1) / 2;
      float yy = y - (aperture - 1) / 2;
      kernel[index] = exp(- (square(xx) + square(yy)) / (2.0 * square(sigma)));
    }
  }

  // ガウシアンフィルタによる平滑化
  vector<float> weight(aperture * aperture);
  vector<float> value(aperture * aperture);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // 外れ値を除去したカーネルの生成
      float sum = 0.0;
      int num = 0;
      for (int fy = 0; fy < aperture; fy++) {
        for (int fx = 0; fx < aperture; fx++) {
          int index = fy * aperture + fx;
          int ix = x + fx - (aperture - 1) / 2;
          int iy = y + fy - (aperture - 1) / 2;
          ix = max(ix, 0);
          ix = min(ix, width - 1);
          iy = max(iy, 0);
          iy = min(iy, height - 1);

          float intensity = src.at<float>(Point(ix, iy));
          if (intensity > outlier) {
            weight[num] = kernel[index];
            value[num] = intensity;
            sum += kernel[index];
            num++;
          }
        }
      }

      // 注目画素の平滑化
      float tmp = 0.0;
      for (int i = 0; i < num; i++) {
        tmp += (weight[i] / sum) * value[i];
      }
      dst.at<float>(Point(x, y)) = tmp;
    }
  }
}

/**
 * @brief バイラテラルフィルタ
 * @param[in]   src 入力画像 32FC1
 * @param[out]  dst 出力画像 32FC1
 * @param[in]   aperture  フィルタの窓サイズ
 * @param[in]   sigma_color 輝度のガウス分布の標準偏差
 * @param[in]   sigma_space 空間のガウス分布の標準偏差
 * @param[in]   outlier   平滑化の計算に含めない外れ値の閾値
 */
void Smooth::bilateralFiler(const Mat src, Mat &dst, int aperture,
    float sigma_color, float sigma_space, float outlier)
{
  Mat tmp = src.clone();
  int width = tmp.cols;
  int height = tmp.rows;
    
  dst = Mat::zeros(height, width, CV_32FC1);

  // バイラテラルフィルタの係数を設定
  vector<float> color_weight(256);
  vector<float> space_weight(aperture * aperture);

  // 空間の係数を設定
  for (int y = 0; y < aperture; y++) {
    for (int x = 0; x < aperture; x++) {
      int index = y * aperture + x;
      float xx = x - (aperture - 1) / 2;
      float yy = y - (aperture - 1) / 2;
      space_weight[index] = exp(- (square(xx) + square(yy)) / (2.0 * square(sigma_space)));
    }
  }

  // バイラテラルフィルタによる平滑化
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      
      // 注目画素の平滑化
      float numer = 0.0, denom = 0.0;
      float target_int = tmp.at<float>(y, x);

      // 深度が欠損値の場合は，輝度値の係数を用いないで値を計算
      if (target_int > outlier) {
        for (int fy = 0; fy < aperture; fy++) {
          for (int fx = 0; fx < aperture; fx++) {
            int index = fy * aperture + fx;
            int ix = x + fx - (aperture - 1) / 2;
            int iy = y + fy - (aperture - 1) / 2;
            ix = max(ix, 0);
            ix = min(ix, width - 1);
            iy = max(iy, 0);
            iy = min(iy, height - 1);

            // 欠損値を除去して値を計算
            float neigh_int = tmp.at<float>(iy, ix);
            if (neigh_int > outlier) {
              float w = space_weight[index] * 
                exp(- square(target_int - neigh_int) / (2.0 * square(sigma_color)));
              //              color_weight[abs(target_int - neigh_int)];

              numer += w;
              denom += neigh_int * w;
            }
          }
        }
      }
      else {
        for (int fy = 0; fy < aperture; fy++) {
          for (int fx = 0; fx < aperture; fx++) {
            int index = fy * aperture + fx;
            int ix = x + fx - (aperture - 1) / 2;
            int iy = y + fy - (aperture - 1) / 2;
            ix = max(ix, 0);
            ix = min(ix, width - 1);
            iy = max(iy, 0);
            iy = min(iy, height - 1);

            // 欠損値を除去して値を計算
            float neigh_int = tmp.at<float>(iy, ix);
            if (neigh_int > outlier) {
              float w = space_weight[index];
              numer += w;
              denom += neigh_int * w;
            }
          }
        }
      }

      if (numer != 0.0) {
        dst.at<float>(y, x) = denom / numer;
      }
    }
  }
}