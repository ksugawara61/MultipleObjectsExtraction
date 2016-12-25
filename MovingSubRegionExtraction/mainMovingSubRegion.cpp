/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainMovingSubRegion.cpp
 *  @brief  移動する部分領域を出力するプログラムに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "../FileOperation/FileOperation.h"
#include "../DrawRegion/DrawRegion.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/FileOperationd.lib")
#pragma comment(lib, "../x64/Debug/DrawRegiond.lib")
#else
#pragma comment(lib, "../x64/Release/FileOperation.lib")
#pragma comment(lib, "../x64/Release/DrawRegion.lib")
#endif

using namespace FileOperation;
using namespace DrawRegion;

/**
 * @brief メイン関数
 * @param[in] argc  引数の数
 * @param[in] argv  引数
 *  (1:移動量の閾値 2:先頭フレーム 3:末尾フレーム 4:横幅 5:縦幅)
 */
int main(int argc, char *argv[])
{
  // 引数の確認
  if (argc < 5) {
    cerr << "usage : moving norm start end width height" << endl;
    return -1;
  }

  // 変数宣言
  float norm = atof(argv[1]);
  int start = atoi(argv[2]);
  int end = atoi(argv[3]);
  int width = atoi(argv[4]);
  int height = atoi(argv[5]);
  Mat label, output;
  Features f;
  char name[80];

  // 移動する対象の部分領域の描画
  for (int i = start; i < end; i++) {
    label = readLabelDat(i, width, height);
    f = readFeaturesDat(i);

    if (label.empty()) {
      break;
    }

    output = drawMovingSubRegion(label, f, norm);
    imshow("output", output);

    string buf;
    sprintf(name, "./out/moving-0%004d.png", i);
    buf = name;
    imwrite(buf, output);
    waitKey(100);
  }

  return 0;
}
