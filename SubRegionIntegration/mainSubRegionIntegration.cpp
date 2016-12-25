/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainSubRegionIntegration.cpp
 *  @brief  領域系列の統合を行うプログラムに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "SubRegionIntegration.h"

#include "../DrawRegion/DrawRegion.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/DrawRegiond.lib")
#else
#pragma comment(lib, "../x64/Release/DrawRegion.lib")
#endif

using namespace DrawRegion;

///* プロトタイプ宣言 */
//void drawIntegrationRegion(SubRegion region, RegionSequences &sequences, 
//  int start, int frame_num, int width, int height);

/**
 * @brief メイン関数
 * @param[in] argc  引数の数 < 8
 * @param[in] argv  引数
 *  (1:領域系列の統合の閾値 2:3次元位置の重み係数 3:シーンフローの重み係数
 *   4:先頭フレーム番号 5:末尾フレーム番号 6:最近点対を利用 
 *   7:領域系列間の接触回数の閾値(0推奨)) 8:横幅 9:縦幅
 */
int main(int argc, char *argv[])
{
  // 引数の確認
  if (argc < 8) {
    cerr << "usage : integration dist w_locate w_flow start end near_flag min_contact width height" << endl;
    return -1;
  }

  // 変数宣言
  char name[80];
  float dist = atof(argv[1]);       // 相違度の閾値
  float w_locate = atof(argv[2]);   // 位置の重み係数
  float w_flow = atof(argv[3]);     // 動き（シーンフロー）の重み係数
  int start = atoi(argv[4]);        // 先頭フレーム番号
  int end = atoi(argv[5]);          // 末尾フレーム番号
  
  // 最近点対を利用
  bool near_flag = true;
  if (atoi(argv[6]) == 0) {
    near_flag = false;
  }

  int min_contact = atoi(argv[7]);

  int width = 640;
  int height = 480;

  if (argc == 10) {
    width = atoi(argv[8]);
    height = atoi(argv[9]);
  }
  else {
    cout << "set default image size 640x480" << endl;
  }

  clock_t s_time, e_time; // 処理時間計測
  vector<Vec3b> rgb;
  rgb.resize(height * width * 30);
  for (int i = 0; i < height * width * 30; i++) {
    rgb[i] = Vec3b(rand() % 256, rand() % 256, rand() % 256);
  }

  // ファイルの読み込み
  RegionSequences region_seq = readRegionSequencesDat(start, end);
  vector<Mat> flows = readMultipleSceneFlowDat(start, end, width, height);  // かなり遅い
  vector<Mat> labels = readMultipleLabelDat(start, end, width, height);

  cout << "start sequence integration process" << endl;
  s_time = clock();

  cout << "prev " << region_seq.size() << endl;

  // 領域系列の統合
  SubRegionIntegration integration(dist, w_locate, w_flow, 
    width, height, start, end, near_flag, min_contact);
  region_seq = integration.integrateSequence(region_seq, flows);

  // ここに接触・分離が行われない領域系列を除去する処理を追加

  // 各フレームでの部分領域の抽出時間を計測
  e_time = clock();
  cout << "next " << region_seq.size() << endl;
  cout << "sequence integration time : " << e_time - s_time << "[ms]" << endl;

  // 統合結果の描画
  /*drawIntegrationRegion(region, region_seq, start, min_sequence, flows.size(),
    width, height);*/
  _mkdir("out_integration");
  for (int t = 0; t < labels.size(); t++) {
    Mat output = drawRegionSequence(getFrameFeatures(region_seq, t), labels.at(t), rgb, t);
    string buf;
    sprintf(name, "./out_integration/integration-0%004d.png", t + start);
    buf = name;
    imwrite(buf, output);
    imshow("output", output);
    waitKey(10);
  }

  cv::destroyAllWindows();

  return 0;
}

///**
// * @brief 領域系列の統合結果を描画
// */
//void drawIntegrationRegion(SubRegion region, RegionSequences &sequences, 
//  int start, int frame_num, int width, int height)
//{
//
//  _mkdir("./out_integration/");
//
//  char name[80];
//
//  vector<Vec3b> rgb(width * height * frame_num);
//  for (int i = 0; i < width * height * frame_num; i++) {
//    rgb.at(i) = Vec3b(rand() % 256, rand() % 256, rand() % 256);
//  }
//
//  // ラベルの読み込み
//  FILE *fpm, *fpf;
//  char readline[MAX_LABEL] = {'\0'};
//
//  for (int i = 0; i < frame_num; i++) {
//    sprintf(name, "./subregion/mlabel-0%004d.dat", i + start);
//    fpm = fopen(name, "r");
//
//    sprintf(name, "./subregion/flabel-0%004d.dat", i + start);
//    fpf = fopen(name, "r");
//
//    Mat mlabel = Mat::zeros(height, width, CV_32SC1);
//    Mat flabel = Mat::zeros(height, width, CV_32SC1);
//
//    for (int y = 0; fgets(readline, MAX_LABEL, fpm) != NULL; y++) {
//      mlabel.at<int>(y, 0) = atoi(strtok(readline, " "));
//      for (int x = 1; x < width; x++) {
//        mlabel.at<int>(y, x) = atoi(strtok(NULL, " "));
//      }
//    }
//
//    for (int y = 0; fgets(readline, MAX_LABEL, fpf) != NULL; y++) {
//      flabel.at<int>(y, 0) = atoi(strtok(readline, " "));
//      for (int x = 1; x < width; x++) {
//        flabel.at<int>(y, x) = atoi(strtok(NULL, " "));
//      }
//    }
//
//    fclose(fpm);
//    fclose(fpf);
//
//    Mat output = ~Mat::zeros(height, width, CV_8UC3);
//
//    for (int j = 0; j < height * width; j++) {
//      int frame = flabel.at<int>(j);
//      int label = mlabel.at<int>(j);
//      
//      for (int k = 0; k < sequences.size(); k++) {
//        if (sequences.at(k).getLabel() == TemporalVertex(frame, label)) {
//          output.at<Vec3b>(j) = 
//            rgb.at((sequences.at(region.find(k)).getLabel().frame + 1) 
//            * sequences.at(region.find(k)).getLabel().label);
//        }
//      }
//    }
//
//    // 保存
//    string buf;
//    sprintf(name, "./out_integration/integration-0%004d.png", i + start);
//    buf = name;
//    imwrite(buf, output);
//
//    imshow("integration", output);
//    waitKey(10);
//  }
//}