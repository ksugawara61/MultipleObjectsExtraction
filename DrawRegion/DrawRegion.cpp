/*****************************************************************************/
/*! @addtogroup 
 *  @file   DrawRegion.cpp
 *  @brief  �`�揈���p���C�u�����Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/
#include "DrawRegion.h"

/**
 * @brief �����̈�ւ̕����i���o�j���ʂ̕`��
 * @param[in] label  �����̈�̃��x��
 * @return  �����̈�̕`�挋��
 */
Mat DrawRegion::drawSubRegion(const Mat &label)
{
  // �ϐ��錾
  int width = label.cols;
  int height = label.rows;
  Mat output(height, width, CV_8UC3);

  vector<Vec3b> rgb(width * height);
  for (int i = 0; i < width * height; i++) {
    rgb.at(i) = Vec3b(rand() % 256, rand() % 256, rand() % 256);
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      output.at<Vec3b>(y, x) = rgb.at(label.at<int>(y, x));
    }
  }

  return output;
}

/**
 * @brief �����̈�̓����ʂ̕`��
 * @param[in] label     �����̈�̃��x��
 * @param[in] boundary  �����̈�̋��E
 * @param[in] f         �����̈�̓����ʂ̃��X�g
 * @param[in] interval  �t���[�̏o�͊Ԋu
 * @param[in] norm      �o�͂���t���[��臒l�i3�����j
 * @return �����̈�̓����ʂ̕`�挋��
 */
Mat DrawRegion::drawSubRegionFeature(const Mat &label, const Mat &boundary,
  const Features &f, const int interval, const float norm)
{
  // �ϐ��錾
  int width = label.cols;
  int height = label.rows;
  Mat output = Mat::zeros(height, width, CV_8UC3);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = label.at<int>(y, x);
      output.at<Vec3b>(y, x) = Vec3b(f.at(index).b, f.at(index).g, f.at(index).r);

      // �����̈�̋��E�̕`��
      int b = boundary.at<uchar>(y, x);
      if (b == 255) {
        line(output, Point(x, y), Point(x, y), Scalar(255, 255, 255));
      }

      // �I�v�e�B�J���t���[�̕`��
      if (y % interval == 0 && x % interval == 0 && f.at(index).norm3d >= norm) { 
        Point3f delta = f.at(index).flow2d;
        line(output, Point(x, y), Point(x + delta.x, y + delta.y), 
          Scalar(0, 0, 255), 1);
      }
    }
  }

  // �d�S�ʒu�̕`��
  int size = f.size();
  for (int i = 0; i < size; i++) {
    Point2f pt = f.at(i).centroid;
    line(output, pt, pt, Scalar(255, 0, 0), 2);
  }

  return output;
}

/**
 * @brief �����̈�ւ̕����i���o�j���ʂ̕`��i�摜���̕��ϒl�ŕ`��j
 * @param[in] label     �����̈�̃��x��
 * @param[in] boundary  �����̈�̋��E
 * @param[in] f         �����̈�̓����ʂ̃��X�g
 * @return �����̈�̕`�挋�ʁiRGB�̕��ϒl�j
 */
Mat DrawRegion::drawSubRegionAverage(const Mat &label, const Mat &boundary,
  const Features &f)
{
  // �ϐ��錾
  int width = label.cols;
  int height = label.rows;
  Mat output = Mat::zeros(height, width, CV_8UC3);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = label.at<int>(y, x);
      output.at<Vec3b>(y, x) = Vec3b(f.at(index).b, f.at(index).g, f.at(index).r);

      // �����̈�̋��E�̕`��
      int b = boundary.at<uchar>(y, x);
      if (b == 255) {
        line(output, Point(x, y), Point(x, y), Scalar(255, 255, 255));
      }
    }
  }

  return output;
}

/**
 * @brief �I�v�e�B�J���t���[�̕`��
 * @param[in] src       �t���[�`��p�̉摜
 * @param[in] flow      �V�[���t���[
 * @param[in] boundary  �����̈�̋��E
 * @param[in] interval  �t���[�̏o�͊Ԋu
 * @param[in] aperture  �����̈�̋��E�t�߂ŏ�������t���[�͈̔�
 * @return  �I�v�e�B�J���t���[�̕`�挋��
 */
Mat DrawRegion::drawOpticalFlow(const Mat &src, const Mat &flow, const Mat &boundary, 
  const int interval, const int aperture)
{
  Mat dst = src.clone();
  int width = dst.cols;
  int height = dst.rows;

  bool boundary_flag;
  int rad = aperture / 2;
  for (int y = 0; y < height; y+=interval) {
    for (int x = 0; x < width; x+=interval) {
      Point3f velocity = flow.at<Point3f>(Point(x, y));

      // ���E�ߖT�̃t���[������
      boundary_flag = false;
      //for (int y2 = y - rad; y2 <= y + rad && y2 < height && 0 <= y2; y2++) {
      //  for (int x2 = x - rad; x2 <= x + rad && x2 < width && 0 <= x2; x2++) {
      //    if (boundary.at<uchar>(Point(x2, y2)) != 0) {
      //      boundary_flag = true;
      //    }
      //  }
      //}

      if ((abs(cvRound(velocity.x)) > 0 || abs(cvRound(velocity.y)) > 0
        || abs(cvRound(velocity.z)) > 0) && !boundary_flag) 
      {
        line(dst, Point(x, y), Point(x + cvRound(velocity.x), y + cvRound(velocity.y)), 
          Scalar(0, 0, 255));
        line(dst, Point(x, y), Point(x, y), Scalar(255, 0, 0), 1);
      }
    }
  }

  return dst;
}

/**
 * @brief �ړ����镔���̈�̐F�t��
 * @param[in] label �����̈�̃��x��
 * @param[in] f     �����̈�̓����ʂ̃��X�g
 * @param[in] norm  �t���[��臒l
 * @return  �ړ����镔���̈�̕`�挋��
 */
Mat DrawRegion::drawMovingSubRegion(const Mat &label, const Features &f, const float norm)
{
  // �ϐ��錾
  int width = label.cols;
  int height = label.rows;
  Mat output = ~Mat::zeros(height, width, CV_8UC3);

  // �t���[�̕`��
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int l = label.at<int>(Point(x, y));
      if (f.at(l).norm3d > norm) {
        output.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
      }
    }
  }

  // �����̈�̋��E��`��
  for (int y = 0; y < height - 1; y++) {
    for (int x = 0; x < width - 1; x++) {
      int comp1 = label.at<int>(y, x);

      for (int yy = y; yy < y + 2; yy++) {
        for (int xx = x; xx < x + 2; xx++) {
          int comp2 = label.at<int>(yy, xx);
          if (comp1 != comp2) {
            output.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
          }
        }
      }
    }
  }

  return output;
}

/**
 * @brief �̈�n��̕`��
 * @param[in] f           �`�悷��̈�n��
 * @param[in] label       �����̈�̃��x��
 * @param[in] rgb         RGB�̃��X�g
 * @param[in] t           �t���[���ԍ�
 * @return  �t���[��t�ł̗̈�n��̕`�挋��
 */
Mat DrawRegion::drawRegionSequence(Features f, 
  const Mat &label, const vector<Vec3b> &rgb, const int t)
{
  int width = label.cols;
  int height = label.rows;

  // �`����̕����̈�������Ƀ\�[�g
  sort(f.begin(), f.end(), featuresLabelAsc);

  Mat output = ~Mat::zeros(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int l = label.at<int>(y, x);

      // �`�悷��̈��񕪒T��
      int low = 0, middle = 0, high = f.size() - 1;
      while(low <= high) {
        middle = (low + high) / 2;

        if (f.at(middle).label == l) {
          output.at<Vec3b>(y, x) = 
            rgb[(f.at(middle).root.frame + 1) * f.at(middle).root.label + 1];
          break;
        }
        else if (f.at(middle).label < l) {
          low = middle + 1;
        }
        else {
          high = middle - 1;
        }
      }
    }
  }

  return output;
}

/**
 * @brief �����̈�̋��E��`��
 * @param[in] label �����̈�̃��x��
 * @param[in] rad   ���E��`�悷��͈�
 */
Mat DrawRegion::drawBoundary(const Mat &label, const int rad)
{
  // �ϐ��錾
  int width = label.cols;
  int height = label.rows;
  Mat output = Mat::zeros(height, width, CV_8UC1);

  // �����̈�̋��E��`��
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp1 = label.at<int>(y, x);

      for (int yy = y - rad; yy < y + rad && yy < height; yy++) {
        for (int xx = x - rad; xx < x + rad && xx < width; xx++) {
          if (yy < 0 || xx < 0) continue;

          int comp2 = label.at<int>(yy, xx);
          if (comp1 != comp2) {
            output.at<uchar>(y, x) = 255;
          }
        }
      }
    }
  }

  return output;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 * @brief �̈�n��̕`��
 * @param[in] region_seq  �̈�n��
 * @param[in] label       �����̈�̃��x��
 * @param[in] rgb         RGB
 * @param[in] t           �t���[���ԍ�
 * @return  �t���[��t�ł̗̈�n��̕`�挋��
 */
//Mat DrawRegion::drawRegionSequence(const RegionSequences &region_seq, 
//  const Mat &label, const vector<Vec3b> &rgb, const int t)
//{
//  int width = label.cols;
//  int height = label.rows;
//
//  Mat output = ~Mat::zeros(height, width, CV_8UC3);
//  for (int y = 0; y < height; y++) {
//    for (int x = 0; x < width; x++) {
//      int l = label.at<int>(y, x);
//
//      int seq_num = region_seq.size();
//      for (int i = 0; i < seq_num; i++) {
//        Features pf = region_seq.at(i).getFeatures(t);
//        for (int j = 0; j < pf.size(); j++) {
//          if (l == pf.at(j).label) {
//            output.at<Vec3b>(y, x) = 
//              rgb[(pf.at(j).root.frame + 1) * pf.at(j).root.label + 1];
//          }
//        }
//      }
//    }
//  }
//
//  return output;
//}
#endif