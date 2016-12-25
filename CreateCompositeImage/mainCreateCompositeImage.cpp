/* 定量的評価の実験に用いる合成画像の生成プログラム */
#include <opencv2\opencv.hpp>
#include <direct.h>

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

int main(int argc, char *argv[])
{
  // 引数の確認
  if (argc < 2) {
    cerr << "usage : createcompositeimage start end" << endl;
    return -1;
  }

  // 変数宣言
  Mat backc, backd, forec, fored, correct, c, d;
  int width, height;
  int start = atoi(argv[1]), end = atoi(argv[2]);
  char name[100];

  // 背景画像・深度の読み込み
  backc = imread("./background-c.bmp", CV_LOAD_IMAGE_COLOR);
  backd = imread("./background-d.bmp", CV_LOAD_IMAGE_GRAYSCALE);

  width = backc.cols;
  height = backc.rows;

  // フォルダの生成
  _mkdir("./composite");

  for (int i = start; i < end; i++) {
    // 正解データの読み込み
    sprintf(name, "./correct/correct-%05d.png", i);
    correct = imread(name, CV_LOAD_IMAGE_COLOR);
    // 前景画像・深度の読み込み
    sprintf(name, "./correct/input-%05d.bmp", i);
    forec = imread(name, CV_LOAD_IMAGE_COLOR);
    sprintf(name, "./correct/depth-%05d.bmp", i);
    fored = imread(name, CV_LOAD_IMAGE_GRAYSCALE);

    if (forec.empty() || fored.empty()) break;

    c = backc.clone();
    d = backd.clone();

    // 画像を比較し，正解データの箇所を背景画像に合成
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        // 合成する箇所か判定
        Vec3b bgr = correct.at<Vec3b>(y, x);
        if (bgr[2] == 255 || bgr[1] == 255 || bgr[0] == 255) {
          if (d.at<uchar>(y, x) > fored.at<uchar>(y, x) || 
            d.at<uchar>(y, x) == 0) 
          {
            c.at<Vec3b>(y, x) = forec.at<Vec3b>(y, x);
            d.at<uchar>(y, x) = fored.at<uchar>(y, x);
          }
        }
      }
    }

    // 合成画像の出力・保存
    imshow("CompositeC", c);
    imshow("CompositeD", d);

    sprintf(name, "./composite/input-%05d.bmp", i);
    imwrite(name, c);
    sprintf(name, "./composite/depth-%05d.bmp", i);
    imwrite(name, d);

    waitKey(10);
  }

  return 0;
}