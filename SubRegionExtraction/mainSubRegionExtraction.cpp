/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainSubRegionExtraction.cpp
 *  @brief  部分領域の抽出を行うプログラムに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include <time.h> // 処理時間計測用
#include "SubRegionExtraction.h"
#include "../FileOperation/FileOperation.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/FileOperationd.lib")
#else
#pragma comment(lib, "../x64/Release/FileOperation.lib")
#endif

using namespace FileOperation;

/**
 * @brief メイン関数
 * @param[in] argc  引数の数 < 12
 * @param[in] argv  引数
 *  (1:正規分布の標準偏差 2:フィルタの窓サイズ 3:部分領域への分割の閾値
 *   4:部分領域の最少画素数 5:画像情報の重み係数 6:深度情報の重み係数
 *   7:運動情報の重み係数 8:輝度の正規分布の標準偏差 9:平滑化の反復回数
 *   10:先頭フレーム番号 11:末尾フレーム番号 12:画像・深度情報のファイル拡張子（.bmp推奨）)
 */
int main(int argc, char *argv[])
{
  // 引数の確認
  if (argc < 8) {
    cerr << "usage : subregionextration sigma ap_filter k min_size w_I w_D w_F " <<
      "color_sigma smooth_iteration " <<
      "start end file_type ap_flow levels iterations scale" << endl;
    cerr << "0.8 3 500 100 1.0 3.0 1.0 20 3 1 10 .bmp 15 3 3 0.5" << endl;
    return -1;
  }

  // 変数宣言
  float sigma = atof(argv[1]);//0.8;  // ガウス分布の標準偏差
  int ap_filter = atoi(argv[2]);//3;  // カーネルの窓サイズ
  float k = atof(argv[3]);//500;      // 閾値の初期値
  int min_size = atoi(argv[4]);//100; // 部分領域の最小サイズ
  float w_I = atof(argv[5]);
  float w_D = atof(argv[6]);
  float w_F = atof(argv[7]);
  float color_sigma = atof(argv[8]);
  int smooth_it = atoi(argv[9]);
  
  int start = atoi(argv[10]);//613;    // フレームの開始番号
  int end = atoi(argv[11]);//713;      // フレームの終了番号
  char *file_type = argv[12];//".bmp"; // 画像の形式

  int ap_flow = 15;   // フローの計算に用いる窓サイズ
  int levels = 3;
  int iterations = 3;
  float scale = 0.5;
  if (argc == 17) {
    int ap_flow = atoi(argv[13]);//15;   // フローの計算に用いる窓サイズ
    int levels = atoi(argv[14]);//3;
    int iterations = atoi(argv[15]);//3;
    float scale = atof(argv[16]);//0.5;
  }
  else {
    cout << "use default flow estimate parameter" << endl;
  }


  clock_t s_time, e_time; // 処理時間計測用
  int total_time = 0;
  FILE *fp, *fp_num;  // 領域数比較用
  fp = fopen("./subregion-time.dat", "w");
  std::fprintf(fp, "# frame time[ms]\n");

  fp_num = fopen("./subregion-num.dat", "w");
  fprintf(fp_num, "# frame num\n");

  char name[80];
  vector<Features> f_vec;
  Mat c_prev, c_curr, c_next;
  Mat d_prev, d_curr, d_next;
  Mat label;
  vector<Mat> boundarys;
  SubRegionExtraction extractor(sigma, ap_filter, k, min_size, w_I, w_D, w_F, 
    color_sigma, smooth_it, ap_flow, levels, iterations, scale);

  _mkdir("./out_subregion");

  s_time = clock();
  for (int i = start; i < end; i++) {
    // 画像・深度情報の読み込み
    string buf;
    sprintf(name, "./data/input-0%004d%s", i, file_type);
    buf = name;
    c_curr = imread(buf);
    sprintf(name, "./data/input-0%004d%s", i + 1, file_type);
    buf = name;
    c_next = imread(buf);

    sprintf(name, "./data/depth-0%004d%s", i, file_type);
    buf = name;
    d_curr = imread(buf, 0);
    sprintf(name, "./data/depth-0%004d%s", i + 1, file_type);
    buf = name;
    d_next = imread(buf, 0);



    // 画像・深度情報を読み込んだか確認
    if (c_curr.empty() || c_next.empty() || d_curr.empty() || d_next.empty()) {
      cerr << "cannot open a color or depth image file." << endl;
      cerr << "please put the input-00000" << file_type << 
        " or depth-00000" << file_type << " in the ./data folder" << endl;
      break;
    }

    // 部分領域の抽出
    Features f;
    //if (label.empty()) {
      f = extractor.subRegionExtract(label, i, c_curr, d_curr,
        c_next, d_next, boundarys);
      f_vec.push_back(f);
    /*}
    else {
      sprintf(name, "./data/input-0%004d%s", i - 1, file_type);
      buf = name;
      c_prev = imread(buf);

      sprintf(name, "./data/depth-0%004d%s", i - 1, file_type);
      buf = name;
      d_prev = imread(buf, 0);

      f = extractor.correctSubRegionExtract(label, i, c_prev, d_prev,
        c_curr, d_curr, c_next, d_next, boundarys);
      f_vec.push_back(f);
    }*/

    // 各フレームでの部分領域の抽出時間を計測
    e_time = clock();
    int tmp = e_time - s_time;
    std::cout << "extract subregion frame No." << i <<  ": " 
      << tmp << "[ms]" << endl;
    std::fprintf(fp, "%d %d\n", i, tmp);
    total_time += tmp;

    fprintf(fp_num, "%d %d\n", i, f.size());

    // 部分領域のラベルと特徴をファイル出力
    writeLabelDat(label, i);
    writeFeaturesDat(f, i);

    // 累積境界を10フレームまで保持
    if (boundarys.size() > 10) {
      boundarys.erase(boundarys.begin());
    }

    s_time = clock();
  }

  std::fprintf(fp, "%d %d\n", 0, total_time);

  fclose(fp);
  fclose(fp_num);

  return 0;
}
