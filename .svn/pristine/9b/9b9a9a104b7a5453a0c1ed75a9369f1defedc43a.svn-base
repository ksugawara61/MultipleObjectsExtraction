/* main.cpp */
#include "../FileOperation/FileOperation.h"
#include "../DrawRegion/DrawRegion.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/FileOperationd.lib")
#else
#pragma comment(lib, "../x64/Release/FileOperation.lib")
#endif

using namespace FileOperation;
using namespace DrawRegion;

int main(int argc, char *argv[])
{
  // 引数の確認
  if (argc < 5) {
    cerr << "usage : moving max_depth start end width height" << endl;
    return -1;
  }

  // 変数宣言
  int threshold = atof(argv[1]);
  int start = atoi(argv[2]);
  int end = atoi(argv[3]);
  int width = atoi(argv[4]);
  int height = atoi(argv[5]);
  char *file_type = ".bmp";
  Mat c, d, output;
  char name[80];

  _mkdir("./correct");

  for (int i = start; i < end; i++) {
    // 深度情報の読み込み
    string buf;
    /*sprintf(name, "./data/input-0%004d%s", i, file_type);
    buf = name;
    c = imread(buf);*/

    sprintf(name, "./data/depth-0%004d%s", i, file_type);
    buf = name;
    d = imread(buf, CV_LOAD_IMAGE_GRAYSCALE);

    if (d.empty()) break;

    output = Mat::zeros(height, width, CV_8UC3);

    // 深度が閾値以上の画素を対象の候補として抽出
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        if (d.at<uchar>(y, x) < threshold && d.at<uchar>(y, x) != 0) {
          output.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
        }
      }
    }

    imshow("output", output);

    sprintf(name, "./correct/correct-0%004d.png", i);
    buf = name;
    imwrite(buf, output);
    waitKey(10);
  }

  return 0;
}
