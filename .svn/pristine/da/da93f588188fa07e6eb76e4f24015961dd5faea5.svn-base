/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainCreateFrameData.cpp
 *  @brief  映像から各フレームの画像・深度情報を生成するプログラムに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

/* main.cpp */
#include <direct.h> // フォルダ作成用
#include <opencv2\opencv.hpp>

#define COLOR_MOVIE "video/color.avi"
#define DEPTH_MOVIE "video/depth.avi"

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
#pragma comment(lib,"opencv_video" CV_VERSION_STR CV_EXT_STR)

using namespace cv;
using namespace std;

#endif

/* プロトタイプ宣言 */
void createFrameData(int start, int end, int interval, float scale, int format,
  int flip_flag);

/**
 * @brief メイン関数
 * @param[in] argc  引数の数 < 6
 * @param[in] argv  引数
 *  (1:先頭フレーム 2:末尾フレーム 3:フレーム間隔 4:スケール
 *  5:ファイルのフォーマット 6:反転フラグ)
 */
int main(int argc, char *argv[])
{
  // 引数の確認
  if (argc < 6) {
    cerr << "usage : createframedata start end interval scale format flip" << endl;
    cerr << "format 0:bmp, 1:png, 2:jpg, 3:ppm" << endl;
    return -1;
  }

  // 各フレームのファイルを生成
  createFrameData(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atof(argv[4]),
    atoi(argv[5]), atoi(argv[6]));

  return 0;
}

/**
 * @brief フレームデータの生成
 * @param[in] start 先頭フレーム
 * @param[in] end   末尾フレーム
 * @param[in] interval  フレーム間隔
 * @param[in] scale フレームのスケール
 * @param[in] format  ファイル生成フォーマット (0:bmp 1:png 2:jpg 3:ppm)
 * @param[in] flip_flag フレームの反転フラグ (1:反転)
 */
void createFrameData(int start, int end, int interval, float scale, int format,
  int flip_flag)
{
  // 変数宣言
  Mat c, d, tmpd, mix;
  char name[100], extension[6];
  int i = 0;

  // ビデオキャプチャ
  VideoCapture capture(COLOR_MOVIE);
  VideoCapture dapture(DEPTH_MOVIE);

  // エラーチェック
  if (!capture.isOpened()) {
    return;
  }
  if (!dapture.isOpened()) {
    return;
  }

  // 拡張子の設定
  if (format == 0) {
    sprintf(extension, ".bmp");
  }
  else if (format == 1) {
    sprintf(extension,".png");
  }
  else if (format == 2) {
    sprintf(extension,".jpg");
  }
  else if (format == 3) {
    sprintf(extension,".ppm");
  }
  else {
    return;
  }

  // フォルダの生成
  _mkdir("./data");

  // フレームデータの生成
  for (capture >> c, dapture >> d; !c.empty() && !d.empty();
    capture >> c, dapture >> d) {
//  for (dapture >> d; !d.empty(); dapture >> d) {
    i++;

    if (i > end) {
      break;
    }
    if (i < start) {
      continue;
    }

    if (i % interval == 0) {
      // フレームのスケールを変更
      resize(c, c, Size(), scale, scale);
      resize(d, d, Size(), scale, scale);

      // フレームの左右反転
      if (flip_flag == 1) {
        flip(c, c, 1);
        flip(d, d, 1);
      }

      // フレームの出力
      sprintf(name, "./data/input-%05d%s", i / interval, extension);
      imwrite(name, c);
      sprintf(name, "./data/depth-%05d%s", i / interval, extension);
      imwrite(name, d);

      // フレームを重ねあわせた結果を出力
//      cvtColor(d, tmpd, CV_GRAY2BGR);
      addWeighted(c, 0.5, d, 0.5, 0, mix);
      sprintf(name, "./data/mix-%05d%s", i / interval, extension);
      imwrite(name, mix);

      cout << "frame " << i << endl;
      imshow("color", c);
      imshow("depth", d);
      imshow("mix", mix);
      waitKey(10);
    }
  }

  // メモリ解放
  capture.release();
  dapture.release();
}