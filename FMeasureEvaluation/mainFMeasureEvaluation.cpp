/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainFMeasureEvaluation.cpp
 *  @brief  実験結果の定量的評価用プログラムに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/
#include <time.h>
#include <direct.h>
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
//  // 引数の確認
//  if (argc < 9) {
//    cerr << "usage : f-measure start end cr cg cb tr tg tb" << endl;
//    return -1;
//  }
//
//  // 変数宣言
//  int start = atoi(argv[1]);
//  int end = atoi(argv[2]);
//  Vec3b correct_bg(255, 255, 255), correct_obj(atoi(argv[5]), atoi(argv[4]), atoi(argv[3]));
//  Vec3b result_bg(255, 255, 255), result_obj(atoi(argv[8]), atoi(argv[7]), atoi(argv[6]));
//  int TP = 0, FP = 0, FN = 0, TN = 0;
//  float precision = 0.0, recall = 0.0, fmeasure = 0.0;  // 評価値
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
//    // 領域抽出結果と正解データの読み込み
//    sprintf(name, "./out_integration/integration-0%004d.png", t);
//    result = imread(name, CV_LOAD_IMAGE_COLOR);
//    sprintf(name, "./correct/correct-0%004d.png", t);
//    correct = imread(name, CV_LOAD_IMAGE_COLOR);
//
//    // 読み込みに失敗した場合処理を終了
//    if (result.empty() || correct.empty()) break;
//
//    width = result.cols;
//    height = result.rows;
//
//    // TP, FP, FN, TNの計算
//    TP = FP = FN = TN = 0;
//    for (int y = 0; y < height; y++) {
//      for (int x = 0; x < width; x++) {
//        Vec3b correct_rgb = correct.at<Vec3b>(y, x);
//        Vec3b &result_rgb = result.at<Vec3b>(y, x);
//
//        // 背景領域を対象とした処理
//        if (correct_rgb == correct_bg) {
//          if (result_rgb != result_obj) {
//            TN++;
//          }
//          else {
//            FP++;
//            sum_pixel++;
//          }
//        }
//        // 物体領域を対象とした処理
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
//        // 出力結果のラベルを正解と同じにする処理（説明しやすくするため）
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
//    // TP, FP, FN, TNの総和を計算
//    sumTP += TP, sumFP += FP, sumFN += FN, sumTN += TN;
//
//    // 各フレームでの領域抽出精度の評価
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
//    // ファイル出力
//    sprintf(name, "./evaluate/result-0%004d.png", t);
//    imwrite(name, result);
//    fprintf(fp, "%d, %f, %f, %f\n", t, precision, recall, fmeasure); 
//  }
//
//  // 時系列全体での領域抽出精度の評価
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
 * @brief メイン関数
 * @param[in]   argc  引数の数 < 12
 * @param[out]  argv  (1:先頭フレーム番号 2:末尾フレーム番号 3,4,5:出力結果の背景のRGB値
 *                     6:対象の物体の数(現時点では2のみ対応) 7,8,9:出力結果の物体1のRGB値
 *                     10,11,12:出力結果の物体2のRGB値)
 */
int main(int argc, char *argv[])
{
  // 引数の確認
  if (argc < 12) {
    cerr << "usage : f-measure start end bgr bgg bgb obj_num r1 g1 b1 r2 g2 b2" << endl;
    return -1;
  }

  // 変数宣言
  int start = atoi(argv[1]);  // 先頭フレーム番号
  int end = atoi(argv[2]);    // 末尾フレーム番号
  Vec3b correct_bg(255, 255, 255), correct_obj1(0, 0, 255), correct_obj2(255, 0, 0);
  Vec3b result_bg(atoi(argv[5]), atoi(argv[4]), atoi(argv[3]));
  int obj_num = atoi(argv[6]);  // 抽出対象の領域の数

  vector<Vec3b> correct_obj, result_obj(obj_num);
  correct_obj.push_back(correct_obj1);
  correct_obj.push_back(correct_obj2);
  for (int i = 0; i < obj_num; i++) {
    int index = i * 3;
    result_obj.at(i) = 
      Vec3b(atoi(argv[9 + index]), atoi(argv[8 + index]), atoi(argv[7 + index]));
  }

  int TP = 0, FP = 0, FN = 0, TN = 0;
  float precision = 0.0, recall = 0.0, fmeasure = 0.0;  // 評価値
  int sumTP = 0, sumFP = 0, sumFN = 0, sumTN = 0;
  float sum_precision = 0.0, sum_recall = 0.0, sum_fmeasure = 0.0;

  char name[80];
  Mat result, correct, out_result;
  int width = 0, height = 0;

  _mkdir("./evaluate");
  sprintf(name, "./evaluate/evaluate.csv");
  FILE *fp = fopen(name, "w");

  for (int t = start; t < end; t++) {
    // 領域抽出結果と正解データの読み込み
    sprintf(name, "./out_integration/integration-0%004d.png", t);
    result = imread(name, CV_LOAD_IMAGE_COLOR);
    sprintf(name, "./correct/correct-0%004d.png", t);
    correct = imread(name, CV_LOAD_IMAGE_COLOR);

    // 読み込みに失敗した場合処理を終了
    if (result.empty() || correct.empty()) break;

    width = result.cols;
    height = result.rows;

    // TP, FP, FN, TNの計算
    TP = FP = FN = TN = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        Vec3b correct_rgb = correct.at<Vec3b>(y, x);
        Vec3b &result_rgb = result.at<Vec3b>(y, x);

        // 背景領域を対象とした処理
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
        // 物体領域を対象とした処理
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

        // 出力結果のラベルを正解と同じにする処理（説明しやすくするため）
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

    // TP, FP, FN, TNの総和を計算
    sumTP += TP, sumFP += FP, sumFN += FN, sumTN += TN;

    // 各フレームでの領域抽出精度の評価
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

    // ファイル出力
    sprintf(name, "./evaluate/result-0%004d.png", t);
    imwrite(name, result);
    fprintf(fp, "%d, %f, %f, %f\n", t, precision, recall, fmeasure); 
  }

  // 時系列全体での領域抽出精度の評価
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
