/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainCreateSequenceImage.cpp
 *  @brief  �̈���΂߂ɂ��炵�ďd�ˍ��킹���摜�𐶐�����v���O�����Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include <opencv2\opencv.hpp>
#include <direct.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// �o�[�W�������̎擾
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// lib�t�@�C�����̍Ō�̕�����Release��Debug�ŕ�����
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif

// ���C�u�����t�@�C��
#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_video" CV_VERSION_STR CV_EXT_STR)

#endif

using namespace std;
using namespace cv;

// �v���g�^�C�v�錾
Mat cutTemporalRegion(int num, const Vec3b bgr, int roi_w, int roi_h, char *file);
Mat mergeTemporalRegion(const vector<Mat> regions, int shift_x = 5, int shift_y = 5);

/**
 * @brief ���C���֐�
 * @param[in] argc  �����̐� < 11
 * @param[in] argv  ����
 *  (1:�擪�t���[�� 2:�����t���[�� 3:�t���[���Ԋu 4:�Ԃ̒l 5:�΂̒l 6:�̒l
 *  7:�e�t���[���Ő؂���͈́i�����j 8:�e�t���[���Ő؂���͈́i�c���j
 *  9:�������ɉ摜�����炷��f�� 10:�c�����ɉ摜�����炷��f��
 *  11:�o�͌��ʂ̃t�@�C���p�X)
 */
int main(int argc, char *argv[])
{
  // �����̊m�F
  if (argc < 11) {
    cerr << "usage : create start end interval r g b roi_w roi_h shift_x shift_y name" << endl;
    cerr << "example : create 1 10 1 10 255 128 100 100 30 30 ./out_matching/matching" << endl;
  }

  int start = atoi(argv[1]);
  int end = atoi(argv[2]) - 1;
  int interval = atoi(argv[3]);
  Vec3b bgr(atoi(argv[6]), atoi(argv[5]), atoi(argv[4]));
  int roi_w = atoi(argv[7]), roi_h = atoi(argv[8]);
  int shift_x = atoi(argv[9]), shift_y = atoi(argv[10]);

  char file[80];
  if (argc == 11) {
    sprintf(file, "integration");
  }
  else {
    sprintf(file, "%s", argv[11]);
  }

  vector<Mat> regions;

  // �e�t���[������Ώۂ̗̈�𒊏o
  char buf[80];
  _mkdir("./result");
  for (int i = end; i >= start; i-=interval) {
    Mat tmp = cutTemporalRegion(i, bgr, roi_w, roi_h, file);
    if (!tmp.empty()) {
      regions.push_back(tmp);
    }

    imshow("output", tmp);
    sprintf(buf, "./result/output-0%004d.png", i);
    imwrite(buf, tmp);
    waitKey(10);
  }

  // �̈�̌n����쐬
  Mat output = mergeTemporalRegion(regions, shift_x, shift_y);
  imwrite("./result/output.png", output);
  cout << "create sequence image" << endl;

  return 0;
}

/**
 * @brief ���n��摜�̕����̈�𒊏o
 * @param[in] num �t���[���ԍ�
 * @param[in] bgr �Ώۂ�RGB��f�l
 * @param[in] roi_w �d�S����Ƃ��Ē��o����͈́i���j
 * @param[in] roi_h �d�S����Ƃ��Ē��o����͈́i�c�j
 * @param[in] file  �t�@�C���̃p�X
 * @return  �e�t���[���̕����̈�̒��o����
 */
Mat cutTemporalRegion(int num, const Vec3b bgr, int roi_w, int roi_h, char *file)
{
  Mat img, segmentation;
  Mat output(roi_h, roi_w, CV_8UC3, Scalar(255, 255, 255));
  char buf[80];
  string name;
//  int top, bottom, left, right;
  Point centroid(0, 0); // �d�S�ʒu
  int size = 0;
  
  sprintf(buf, "./data/input-0%004d.bmp", num);
  name = buf;
  img = imread(name, CV_LOAD_IMAGE_COLOR);
  sprintf(buf, "%s-0%004d.png", file, num);
  name = buf;
  segmentation = imread(name);

  if (img.empty() || segmentation.empty()) {
    cerr << "cannot read image" << endl;
    return img;
  }

  //top = img.rows;
  //bottom = 0;
  //left = img.cols;
  //right = 0;

  // �d�S�ʒu�̌v�Z
  for (int y = 0; y < img.rows; y++) {
    for (int x = 0; x < img.cols; x++) {
      Vec3b temp = segmentation.at<Vec3b>(y, x);
      if (temp == bgr) {
        centroid.x += x;
        centroid.y += y;
        size++;

        //if (top > y) {
        //  top = y;
        //}
        //if (bottom < y) {
        //  bottom = y;
        //}

        //if (left > x) {
        //  left = x;
        //}
        //if (right < x) {
        //  right = x;
        //}
//        output.at<Vec3b>(Point(x, y)) = img.at<Vec3b>(y, x);
      }
    }
  }

  // �Ώۗ̈悪�Ȃ��ꍇ�C��i���j�̏�Ԃŏo��
  if (size == 0) {
    return output;
  }

  centroid.x /= size;
  centroid.y /= size;

  //if (top > bottom || left > right) {
  //  cerr << "cannot extract target region" << endl;
  //  return img;
  //}
  
  /*Mat output(bottom - top + 1, right - left + 1, CV_8UC3, Scalar(255, 255, 255));
  for (int y = top; y <= bottom; y++) {
    for (int x = left; x <= right; x++) {
      if (segmentation.at<Vec3b>(y, x) == bgr) {
        output.at<Vec3b>(y - top, x - left) = img.at<Vec3b>(y, x);
      }
    }
  }*/

  // �d�S�ʒu����ɗ̈�ɑ������f��F�t��
  for (int y = centroid.y - roi_h / 2; y < centroid.y + roi_h / 2 && y < img.rows; y++)  {
    if (y < 0) y = 0;
    for (int x = centroid.x - roi_w / 2; x < centroid.x + roi_w / 2 && x < img.cols; x++)  {
      if (x < 0) x = 0;
      if (segmentation.at<Vec3b>(y, x) == bgr) {
        output.at<Vec3b>(y - (centroid.y - roi_h / 2), x - (centroid.x - roi_w / 2))
          = img.at<Vec3b>(y, x);
      }
    }
  }

  return output;
}

/**
 * @brief �e�����̈���΂߂ɂ��炵�ďo��
 * @param[in] regions ���o���ʂ̌n��
 * @param[in] shift_x x�����ɂ��炷��f��
 * @param[in] shift_y y�����ɂ��炷��f��
 * @return  ���o���ʂ��΂߂ɂ��炵�ďd�ˍ��킹���摜
 */
Mat mergeTemporalRegion(const vector<Mat> regions, int shift_x, int shift_y)
{
  if (regions.empty()) {
    Mat output;
    return output;
  }

  int frame_num = regions.size();

  Mat output(regions.at(0).rows + abs(shift_y) * frame_num, 
    regions.at(0).cols + abs(shift_x) * frame_num, CV_8UC3, Scalar(255, 255, 255));
  Vec3b white(255, 255, 255);

  //for (int i = 0; i < frame_num; i++) {
  //  for (int y = 0; y < regions.at(i).rows; y++) {
  //    for (int x = 0; x < regions.at(i).cols; x++) {
  //      if (regions.at(i).at<Vec3b>(y, x) != white) {
  //        output.at<Vec3b>(y + shift_y * i, x + shift_x * i)
  //          = regions.at(i).at<Vec3b>(y, x);
  //      }
  //    }
  //  }
  //}

  for (int i = 0; i < frame_num; i++) {
    for (int y = 0; y < regions.at(i).rows; y++) {
      for (int x = 0; x < regions.at(i).cols; x++) {
        if (regions.at(i).at<Vec3b>(y, x) != white) {
          output.at<Vec3b>(y + shift_y * i, x + shift_x * (frame_num - i - 1))
            = regions.at(i).at<Vec3b>(y, x);
        }
      }
    }
  }

  return output;
}