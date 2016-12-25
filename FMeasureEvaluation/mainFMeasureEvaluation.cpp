/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainFMeasureEvaluation.cpp
 *  @brief  �������ʂ̒�ʓI�]���p�v���O�����Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/
#include <time.h>
#include <direct.h>
#include <opencv2\opencv.hpp>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// �o�[�W�������̎擾
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// lib�t�@�C�����̍Ō�̕�����Release��Debug�ŕ�����
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif

#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)

#endif

using namespace std;
using namespace cv;

/**
 * 20150204
 */
//int main(int argc, char *argv[])
//{
//  // �����̊m�F
//  if (argc < 9) {
//    cerr << "usage : f-measure start end cr cg cb tr tg tb" << endl;
//    return -1;
//  }
//
//  // �ϐ��錾
//  int start = atoi(argv[1]);
//  int end = atoi(argv[2]);
//  Vec3b correct_bg(255, 255, 255), correct_obj(atoi(argv[5]), atoi(argv[4]), atoi(argv[3]));
//  Vec3b result_bg(255, 255, 255), result_obj(atoi(argv[8]), atoi(argv[7]), atoi(argv[6]));
//  int TP = 0, FP = 0, FN = 0, TN = 0;
//  float precision = 0.0, recall = 0.0, fmeasure = 0.0;  // �]���l
//  int sumTP = 0, sumFP = 0, sumFN = 0, sumTN = 0;
//  float sum_precision = 0.0, sum_recall = 0.0, sum_fmeasure = 0.0;
//  int sum_pixel = 0;
//
//  char name[80];
//  Mat result, correct, out_result;
//  int width = 0, height = 0;
//
//  _mkdir("./evaluate");
//  sprintf(name, "./evaluate/evaluate.csv");
//  FILE *fp = fopen(name, "w");
//
//  for (int t = start; t < end; t++) {
//    // �̈撊�o���ʂƐ����f�[�^�̓ǂݍ���
//    sprintf(name, "./out_integration/integration-0%004d.png", t);
//    result = imread(name, CV_LOAD_IMAGE_COLOR);
//    sprintf(name, "./correct/correct-0%004d.png", t);
//    correct = imread(name, CV_LOAD_IMAGE_COLOR);
//
//    // �ǂݍ��݂Ɏ��s�����ꍇ�������I��
//    if (result.empty() || correct.empty()) break;
//
//    width = result.cols;
//    height = result.rows;
//
//    // TP, FP, FN, TN�̌v�Z
//    TP = FP = FN = TN = 0;
//    for (int y = 0; y < height; y++) {
//      for (int x = 0; x < width; x++) {
//        Vec3b correct_rgb = correct.at<Vec3b>(y, x);
//        Vec3b &result_rgb = result.at<Vec3b>(y, x);
//
//        // �w�i�̈��ΏۂƂ�������
//        if (correct_rgb == correct_bg) {
//          if (result_rgb != result_obj) {
//            TN++;
//          }
//          else {
//            FP++;
//            sum_pixel++;
//          }
//        }
//        // ���̗̈��ΏۂƂ�������
//        else if (correct_rgb == correct_obj) {
//          if (result_rgb == result_obj) {
//            TP++;
//            sum_pixel++;
//          }
//          else {
//            FN++;
//          }
//        }
//        else if (result_rgb == result_obj) {
//          FP++;
//        }
//        else {
//          TN++;
//        }
//
//        // �o�͌��ʂ̃��x���𐳉��Ɠ����ɂ��鏈���i�������₷�����邽�߁j
//        if (result_rgb == result_bg) {
//          result_rgb = correct_rgb = Vec3b(255, 255, 255);
//        }
//        else {
//          if (result_rgb == result_obj) {
//            result_rgb = correct_obj;
//          }
//        }
//      }
//    }
//
//    // TP, FP, FN, TN�̑��a���v�Z
//    sumTP += TP, sumFP += FP, sumFN += FN, sumTN += TN;
//
//    // �e�t���[���ł̗̈撊�o���x�̕]��
//    precision = TP / (((TP + FP) != 0) ? (TP + FP) : 0.001);
//    recall = TP / (((TP + FN) != 0) ? (TP + FN) : 0.001);
//    fmeasure = (((recall + precision) != 0) ? 
//      2.0 * recall * precision / (recall + precision) : 0);
//
//    cout << "frame " << t << " precision : " << precision << endl;
//    cout << "frame " << t << " recall : " << recall << endl;
//    cout << "frame " << t << " F-measure : " << fmeasure<< endl;
//    cout << endl;
//
//    imshow("correct", correct);
//    imshow("result", result);
//    waitKey(10);
//
//    // �t�@�C���o��
//    sprintf(name, "./evaluate/result-0%004d.png", t);
//    imwrite(name, result);
//    fprintf(fp, "%d, %f, %f, %f\n", t, precision, recall, fmeasure); 
//  }
//
//  // ���n��S�̂ł̗̈撊�o���x�̕]��
//  sum_precision = sumTP / (((sumTP + sumFP) != 0) ? (sumTP + sumFP) : 0.001);
//  sum_recall = sumTP / (((sumTP + sumFN) != 0) ? (sumTP + sumFN) : 0.001);
//  sum_fmeasure = (((sum_recall + sum_precision) != 0) ?
//    2 * sum_recall * sum_precision / (sum_recall + sum_precision) : 0);
//
//  cout << "precision : " << sum_precision << endl;
//  cout << "recall : " << sum_recall << endl;
//  cout << "F-measure : " << sum_fmeasure<< endl;
//
//  fprintf(fp, "%d, %f, %f, %f\n", sum_pixel, sum_precision, sum_recall, sum_fmeasure);
//  fclose(fp);
//
//  return 0;
//}

/**
 * @brief ���C���֐�
 * @param[in]   argc  �����̐� < 12
 * @param[out]  argv  (1:�擪�t���[���ԍ� 2:�����t���[���ԍ� 3,4,5:�o�͌��ʂ̔w�i��RGB�l
 *                     6:�Ώۂ̕��̂̐�(�����_�ł�2�̂ݑΉ�) 7,8,9:�o�͌��ʂ̕���1��RGB�l
 *                     10,11,12:�o�͌��ʂ̕���2��RGB�l)
 */
int main(int argc, char *argv[])
{
  // �����̊m�F
  if (argc < 12) {
    cerr << "usage : f-measure start end bgr bgg bgb obj_num r1 g1 b1 r2 g2 b2" << endl;
    return -1;
  }

  // �ϐ��錾
  int start = atoi(argv[1]);  // �擪�t���[���ԍ�
  int end = atoi(argv[2]);    // �����t���[���ԍ�
  Vec3b correct_bg(255, 255, 255), correct_obj1(0, 0, 255), correct_obj2(255, 0, 0);
  Vec3b result_bg(atoi(argv[5]), atoi(argv[4]), atoi(argv[3]));
  int obj_num = atoi(argv[6]);  // ���o�Ώۂ̗̈�̐�

  vector<Vec3b> correct_obj, result_obj(obj_num);
  correct_obj.push_back(correct_obj1);
  correct_obj.push_back(correct_obj2);
  for (int i = 0; i < obj_num; i++) {
    int index = i * 3;
    result_obj.at(i) = 
      Vec3b(atoi(argv[9 + index]), atoi(argv[8 + index]), atoi(argv[7 + index]));
  }

  int TP = 0, FP = 0, FN = 0, TN = 0;
  float precision = 0.0, recall = 0.0, fmeasure = 0.0;  // �]���l
  int sumTP = 0, sumFP = 0, sumFN = 0, sumTN = 0;
  float sum_precision = 0.0, sum_recall = 0.0, sum_fmeasure = 0.0;

  char name[80];
  Mat result, correct, out_result;
  int width = 0, height = 0;

  _mkdir("./evaluate");
  sprintf(name, "./evaluate/evaluate.csv");
  FILE *fp = fopen(name, "w");

  for (int t = start; t < end; t++) {
    // �̈撊�o���ʂƐ����f�[�^�̓ǂݍ���
    sprintf(name, "./out_integration/integration-0%004d.png", t);
    result = imread(name, CV_LOAD_IMAGE_COLOR);
    sprintf(name, "./correct/correct-0%004d.png", t);
    correct = imread(name, CV_LOAD_IMAGE_COLOR);

    // �ǂݍ��݂Ɏ��s�����ꍇ�������I��
    if (result.empty() || correct.empty()) break;

    width = result.cols;
    height = result.rows;

    // TP, FP, FN, TN�̌v�Z
    TP = FP = FN = TN = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        Vec3b correct_rgb = correct.at<Vec3b>(y, x);
        Vec3b &result_rgb = result.at<Vec3b>(y, x);

        // �w�i�̈��ΏۂƂ�������
        if (correct_rgb == correct_bg) {
          bool flag = true;
          for (int i = 0; i < obj_num; i++) {
            if (result_rgb == result_obj.at(i)) {
              FP++;
              flag = false;
            }
          }
          if (flag) {
            TN++;
          }
          //if (result_rgb != result_bg) {
          //  FP++;
          //}
          //else {
          //  TN++;
          //}
        }
        // ���̗̈��ΏۂƂ�������
        else {
          for (int i = 0; i < obj_num; i++) {
            if (correct_rgb == correct_obj.at(i)) {
              if (result_rgb == result_obj.at(i)) {
                TP++;
              }
              else {
                FN++;
              }
            }
          }
        }

        // �o�͌��ʂ̃��x���𐳉��Ɠ����ɂ��鏈���i�������₷�����邽�߁j
        bool flag = true;
        if (result_rgb == result_bg) {
          result_rgb = correct_rgb = Vec3b(255, 255, 255);
        }
        else {
          for (int i = 0; i < obj_num; i++) {
            if (result_rgb == result_obj.at(i)) {
              result_rgb = correct_obj.at(i);
              flag = false;
            }
          }
          if (flag) {
            correct_rgb = Vec3b(255, 255, 255);
          }
          else {
            if (correct_rgb == Vec3b(0, 0, 0)) {
              correct_rgb = Vec3b(255, 255, 255);
            }
          }
        }
      }
    }

    // TP, FP, FN, TN�̑��a���v�Z
    sumTP += TP, sumFP += FP, sumFN += FN, sumTN += TN;

    // �e�t���[���ł̗̈撊�o���x�̕]��
    precision = TP / (((TP + FP) != 0) ? (TP + FP) : 0.001);
    recall = TP / (((TP + FN) != 0) ? (TP + FN) : 0.001);
    fmeasure = (((recall + precision) != 0) ? 
      2.0 * recall * precision / (recall + precision) : 0);

    cout << "frame " << t << " precision : " << precision << endl;
    cout << "frame " << t << " recall : " << recall << endl;
    cout << "frame " << t << " F-measure : " << fmeasure<< endl;
    cout << endl;

    imshow("correct", correct);
    imshow("result", result);
    waitKey(10);

    // �t�@�C���o��
    sprintf(name, "./evaluate/result-0%004d.png", t);
    imwrite(name, result);
    fprintf(fp, "%d, %f, %f, %f\n", t, precision, recall, fmeasure); 
  }

  // ���n��S�̂ł̗̈撊�o���x�̕]��
  sum_precision = sumTP / (((sumTP + sumFP) != 0) ? (sumTP + sumFP) : 0.001);
  sum_recall = sumTP / (((sumTP + sumFN) != 0) ? (sumTP + sumFN) : 0.001);
  sum_fmeasure = (((sum_recall + sum_precision) != 0) ?
    2 * sum_recall * sum_precision / (sum_recall + sum_precision) : 0);

  cout << "precision : " << sum_precision << endl;
  cout << "recall : " << sum_recall << endl;
  cout << "F-measure : " << sum_fmeasure<< endl;

  fprintf(fp, "?%d, ?%f, ?%f, ?%f\n", 0, sum_precision, sum_recall, sum_fmeasure);
  fclose(fp);

  return 0;
}
