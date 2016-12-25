/*****************************************************************************/
/*! @addtogroup 
 *  @file   FileOperationRead.cpp
 *  @brief  ファイル操作（読み込み）に関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/
#include "FileOperation.h"

/**
 * @brief 部分領域のラベルファイルの読み込み
 * @param[in] num フレーム番号
 * @param[in] width   フレーム横幅
 * @param[in] height  フレーム縦幅
 * @return ラベル
 */
Mat FileOperation::readLabelDat(const int num, const int width, const int height)
{
  char name[80];
  FILE *fp;
  char readline[MAX_LABEL] = {'\0'};

  sprintf(name, "./subregion/label-0%004d.dat", num);
  fp = fopen(name, "r");

  Mat label = Mat::zeros(height, width, CV_32SC1);
  for (int y = 0; fgets(readline, MAX_LABEL, fp) != NULL; y++) {
    label.at<int>(y, 0) = atoi(strtok(readline, " "));
    for (int x = 1; x < width; x++) {
      label.at<int>(y, x) = atoi(strtok(NULL, " "));
    }
  }

  fclose(fp);

  return label;
}

/**
 * @brief 部分領域のラベルファイルのリストの読み込み
 * @param[in] start 開始フレーム番号
 * @param[in] end   終了フレーム番号
 * @param[in] width   フレーム横幅
 * @param[in] height  フレーム縦幅
 * @return 部分領域のラベルのリスト
 */
vector<Mat> FileOperation::readMultipleLabelDat(const int start, const int end, const int width,
  const int height)
{
  vector<Mat> labels;

  for (int i = start; i < end; i++) {
    labels.push_back(readLabelDat(i, width, height));
  }

  return labels;
}

/**
 * @brief 部分領域の特徴量のファイル読み込み
 * @param[in] num フレーム番号
 * @return フレームnumの部分領域の特徴量
 */
Features FileOperation::readFeaturesDat(const int num)
{
  char name[80];
  FILE *fp;
  char readline[MAX_FEATURE] = {'\0'};
  Features features;

  sprintf(name, "./subregion/feature-0%004d.dat", num);
  fp = fopen(name, "r");

  while(fgets(readline, MAX_FEATURE, fp) != NULL) {
    Feature f;

    f.num = atoi(strtok(readline, " "));
    f.label = atoi(strtok(NULL, " "));
    f.centroid.x = atof(strtok(NULL, " "));
    f.centroid.y = atof(strtok(NULL, " "));
    f.centroid3d.x = atof(strtok(NULL, " "));
    f.centroid3d.y = atof(strtok(NULL, " "));
    f.centroid3d.z = atof(strtok(NULL, " "));
    f.flow2d.x = atof(strtok(NULL, " "));
    f.flow2d.y = atof(strtok(NULL, " "));
    f.flow2d.z = atof(strtok(NULL, " "));
    f.flow3d.x = atof(strtok(NULL, " "));
    f.flow3d.y = atof(strtok(NULL, " "));
    f.flow3d.z = atof(strtok(NULL, " "));
    f.r = atof(strtok(NULL, " "));
    f.g = atof(strtok(NULL, " "));
    f.b = atof(strtok(NULL, " "));
    f.d = atof(strtok(NULL, " "));
    f.norm2d = atof(strtok(NULL, " "));
    f.norm3d = atof(strtok(NULL, " "));
    f.size = atof(strtok(NULL, " "));

    features.push_back(f);
  }

  fclose(fp);

  return features;
}

/**
 * @brief 部分領域の特徴量のリストのファイル読み込み
 * @param[in] start 先頭フレーム番号
 * @param[in] end   末尾フレーム番号
 * @return 部分領域の特徴量のリスト
 */
vector<Features> FileOperation::readMultipleFeaturesDat(const int start, const int end)
{
  vector<Features> f_vec;
  for (int i = start; i < end; i++) {
    f_vec.push_back(readFeaturesDat(i));
  }

  return f_vec;
}

/**
 * @brief シーンフローのファイル読み込み
 * @param[in] num フレームの番号
 * @param[in] width   フレーム横幅
 * @param[in] height  フレーム縦幅
 * @return  フレームnumのシーンフロー
 */
Mat FileOperation::readSceneFlowDat(const int num, const int width, const int height)
{
   char name[80];
  FILE *fp;
  char readline[MAX_LABEL] = {'\0'};
  Mat flow = Mat::zeros(height, width, CV_32FC3);
  int x, y;
  Point3f delta;

  // シーンフローの読み込み
  sprintf(name, "./subregion/sceneflow-0%004d.dat", num);
  fp = fopen(name, "r");

  while(fgets(readline, MAX_LABEL, fp) != NULL) {
    x = atoi(strtok(readline, " "));
    y = atoi(strtok(NULL, " "));
    delta.x = atof(strtok(NULL, " "));
    delta.y = atof(strtok(NULL, " "));
    delta.z = atof(strtok(NULL, " "));

    flow.at<Point3f>(y, x) = delta;
  }

  fclose(fp);

  return flow;


  //Mat flow;
  //vector<Mat> f;
  //Mat dx = Mat::zeros(height, width, CV_32FC1);
  //Mat dy = Mat::zeros(height, width, CV_32FC1);
  //Mat dz = Mat::zeros(height, width, CV_32FC1);
  //float delta;

  //// シーンフローの各成分をファイルに書き込み
  //sprintf(name, "./subregion/dx-0%004d.dat", num);
  //fp = fopen(name, "r");
  //for (int y = 0; fgets(readline, MAX_LABEL, fp) != NULL; y++) 
  //{
  //  delta = atof(strtok(readline, " "));

  //  dx.at<float>(y, 0) = delta;

  //  for (int x = 1; x < width; x++) {
  //    delta = atof(strtok(NULL, " "));
  //    dx.at<float>(y, x) = delta;
  //  }
  //}
  //fclose(fp);

  //sprintf(name, "./subregion/dy-0%004d.dat", num);
  //fp = fopen(name, "r");
  //for (int y = 0; fgets(readline, MAX_LABEL, fp) != NULL; y++) 
  //{
  //  delta = atof(strtok(readline, " "));

  //  dy.at<float>(y, 0) = delta;

  //  for (int x = 1; x < width; x++) {
  //    delta = atof(strtok(NULL, " "));
  //    dy.at<float>(y, x) = delta;
  //  }
  //}
  //fclose(fp);

  //sprintf(name, "./subregion/dz-0%004d.dat", num);
  //fp = fopen(name, "r");
  //for (int y = 0; fgets(readline, MAX_LABEL, fp) != NULL; y++) 
  //{
  //  delta = atof(strtok(readline, " "));

  //  dz.at<float>(y, 0) = delta;

  //  for (int x = 1; x < width; x++) {
  //    delta = atof(strtok(NULL, " "));
  //    dz.at<float>(y, x) = delta;
  //  }
  //}
  //fclose(fp);

  //f.push_back(dx);
  //f.push_back(dy);
  //f.push_back(dz);

  //cv::merge(f, flow);
    
  //return flow;
}

/**
 * @brief シーンフローのリストのファイル読み込み
 * @param[in] start   先頭フレーム番号
 * @param[in] end     末尾フレーム番号
 * @param[in] width   フレーム横幅
 * @param[in] height  フレーム縦幅
 * @return  シーンフローのリスト
 */
vector<Mat> FileOperation::readMultipleSceneFlowDat(const int start, const int end, 
  const int width, const int height)
{
  vector<Mat> flows;

  for (int i = start; i < end; i++) {
    flows.push_back(readSceneFlowDat(i, width, height));
  }

  return flows;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
///**
// * @brief 部分領域の系列の読み込み
// * @return 部分領域の系列の配列
// */
//vector<FeatureSequence> FileOperation::readMultipleSequencesDat(void)
//{
//  FILE *fp;
//  char name[80];
//  char readline[MAX_FEATURE] = {'\0'};
//  vector<FeatureSequence> sequences;
//
//  int frame, label, length;
//  for (int i = 0; ;i++) {
//    // 系列全体の特徴を書き込み
//    sprintf(name, "./sequences/subregion-%d.dat", i);
//    if ((fp = fopen(name, "r")) == NULL) {
//      break;
//    }
//
//    fgets(readline, MAX_FEATURE, fp);
//    frame = atoi(strtok(readline, " "));
//    label = atoi(strtok(NULL, " "));
//    length = atoi(strtok(NULL, " "));
//
//    FeatureSequence s(frame, label, length);
//
//    for (int j = 0; j < length /*fgets(readline, MAX_FEATURE, fp) != NULL*/; j++) {
//      fgets(readline, MAX_FEATURE, fp);
//
//      Feature f;
//
//      f.num = atoi(strtok(readline, " "));
//      f.centroid.x = atof(strtok(NULL, " "));
//      f.centroid.y = atof(strtok(NULL, " "));
//      f.centroid3d.x = atof(strtok(NULL, " "));
//      f.centroid3d.y = atof(strtok(NULL, " "));
//      f.centroid3d.z = atof(strtok(NULL, " "));
//      f.flow2d.x = atof(strtok(NULL, " "));
//      f.flow2d.y = atof(strtok(NULL, " "));
//      f.flow2d.z = atof(strtok(NULL, " "));
//      f.flow3d.x = atof(strtok(NULL, " "));
//      f.flow3d.y = atof(strtok(NULL, " "));
//      f.flow3d.z = atof(strtok(NULL, " "));
//      f.r = atof(strtok(NULL, " "));
//      f.g = atof(strtok(NULL, " "));
//      f.b = atof(strtok(NULL, " "));
//      f.d = atof(strtok(NULL, " "));
//      f.norm2d = atof(strtok(NULL, " "));
//      f.norm3d = atof(strtok(NULL, " "));
//      f.size = atof(strtok(NULL, " "));
//
//      int cnt = 0;
//      for (int k = atoi(strtok(NULL, " ")); k > 0; k--) {
//        Feature tmp = f;
//        tmp.label = atoi(strtok(NULL, " "));
//        if (cnt != 0) {
//          tmp.size = 0;
//        }
//        s.assign(j, tmp);
//        cnt++;
//      }
//
//      s.average(j);
//    }
//
//    fgets(readline, MAX_FEATURE, fp);
//    s.setLength(atoi(readline));
//
//    sequences.push_back(s);
//    
//    fclose(fp);
//  }
//
//  return sequences;
//}
//
///**
// * @brief 部分領域の系列の読み込み
// * @return 部分領域の系列の配列
// * @param[in] start 対応付け処理の最初のフレーム
// * @param[in] end   対応付け処理の最後のフレーム
// */
//vector<FeatureSequence> FileOperation::readMultipleSequencesDat(const int start, const int end)
//{
//  FILE *fp;
//  char name[80];
//  char readline[MAX_FEATURE] = {'\0'};
//  vector<FeatureSequence> sequences;
//
//  int frame, label, length;
//
//  for (int i = 0; ;i++) {
//    // 系列全体の特徴を書き込み
//    sprintf(name, "./%d-%d/subregion-%d.dat", start, end - 1, i);
//    if ((fp = fopen(name, "r")) == NULL) {
//      break;
//    }
//
//    fgets(readline, MAX_FEATURE, fp);
//    frame = atoi(strtok(readline, " "));
//    label = atoi(strtok(NULL, " "));
//    length = atoi(strtok(NULL, " "));
//
//    FeatureSequence s(frame, label, end - start);
//
//    for (int j = 0; j < end - start/*fgets(readline, MAX_FEATURE, fp) != NULL*/; j++) {
//      fgets(readline, MAX_FEATURE, fp);
//      
//      Feature f;
//
//      f.num = atoi(strtok(readline, " "));
//      f.centroid.x = atof(strtok(NULL, " "));
//      f.centroid.y = atof(strtok(NULL, " "));
//      f.centroid3d.x = atof(strtok(NULL, " "));
//      f.centroid3d.y = atof(strtok(NULL, " "));
//      f.centroid3d.z = atof(strtok(NULL, " "));
//      f.flow2d.x = atof(strtok(NULL, " "));
//      f.flow2d.y = atof(strtok(NULL, " "));
//      f.flow2d.z = atof(strtok(NULL, " "));
//      f.flow3d.x = atof(strtok(NULL, " "));
//      f.flow3d.y = atof(strtok(NULL, " "));
//      f.flow3d.z = atof(strtok(NULL, " "));
//      f.r = atof(strtok(NULL, " "));
//      f.g = atof(strtok(NULL, " "));
//      f.b = atof(strtok(NULL, " "));
//      f.d = atof(strtok(NULL, " "));
//      f.norm2d = atof(strtok(NULL, " "));
//      f.norm3d = atof(strtok(NULL, " "));
//      f.size = atof(strtok(NULL, " "));
//
//      int cnt = 0;
//      for (int k = atoi(strtok(NULL, " ")); k > 0; k--) {
//        Feature tmp = f;
//        tmp.label = atoi(strtok(NULL, " "));
//        if (cnt != 0) {
//          tmp.size = 0;
//        }
//        s.assign(j, tmp);
//        cnt++;
//      }
//
//      s.average(j);
//    }
//
//    fgets(readline, MAX_FEATURE, fp);
//    s.setLength(atoi(readline));
//
//    sequences.push_back(s);
//    
//    fclose(fp);
//  }
//
//  if (sequences.size() == 0) {
//    cerr << "cannot exist " << "/" << start << "-" << end - 1 << "/" << endl;
//  }
//
//
//  return sequences;
//}

#endif

/**
 * @brief 領域系列のリストのファイル読み込み
 * @param[in] start 先頭フレーム
 * @param[in] end   末尾フレーム
 * @return  領域系列のリスト
 */
RegionSequences FileOperation::readRegionSequencesDat(const int start, const int end)
{
  FILE *fp;
  char name[80];
  char readline[MAX_FEATURE] = {'\0'};
  RegionSequences region_seq;

  int frame, label, length;

  for (int i = 0; ;i++) {
    // 系列全体の特徴を書き込み
    sprintf(name, "./%d-%d/subregion-%d.dat", start, end - 1, i);
    if ((fp = fopen(name, "r")) == NULL) {
      break;
    }

    fgets(readline, MAX_FEATURE, fp);
    frame = atoi(strtok(readline, " "));
    label = atoi(strtok(NULL, " "));
    length = atoi(strtok(NULL, " "));
    TemporalVertex root(frame, label);

    RegionSequence tmp(root, end - start);

    // 領域系列の特徴量の格納
    while(fgets(readline, MAX_FEATURE, fp) != NULL) {
      Feature f;

      f.num = atoi(strtok(readline, " "));
      f.label = atoi(strtok(NULL, " "));
      f.root = root;
      f.index = i;
      f.centroid.x = atof(strtok(NULL, " "));
      f.centroid.y = atof(strtok(NULL, " "));
      f.centroid3d.x = atof(strtok(NULL, " "));
      f.centroid3d.y = atof(strtok(NULL, " "));
      f.centroid3d.z = atof(strtok(NULL, " "));
      f.flow2d.x = atof(strtok(NULL, " "));
      f.flow2d.y = atof(strtok(NULL, " "));
      f.flow2d.z = atof(strtok(NULL, " "));
      f.flow3d.x = atof(strtok(NULL, " "));
      f.flow3d.y = atof(strtok(NULL, " "));
      f.flow3d.z = atof(strtok(NULL, " "));
      f.r = atof(strtok(NULL, " "));
      f.g = atof(strtok(NULL, " "));
      f.b = atof(strtok(NULL, " "));
      f.d = atof(strtok(NULL, " "));
      f.norm2d = atof(strtok(NULL, " "));
      f.norm3d = atof(strtok(NULL, " "));
      f.size = atof(strtok(NULL, " "));

      tmp.setFeature(f.num, f);
    }
    tmp.calcNorm();

    region_seq.push_back(tmp);
    
    fclose(fp);
  }

  if (region_seq.size() == 0) {
    cerr << "cannot exist " << "/" << start << "-" << end - 1 << "/" << endl;
  }


  return region_seq;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
///**
// * @brief
// * @param[in] num   フレーム番号
// * @param[in] subregion_num 移動する部分領域の数
// * @return  部分領域間の距離
// */
//Mat FileOperation::readDistanceDat(const int num, const int subregion_num)
//{
//  FILE *fp ;
//  char name[80];
//  char readline[MAX_FEATURE] = {'\0'};
//  Mat d;
//
//  // 系列全体の特徴を読み込み
//  sprintf(name, "./sequences/subregiondistance-%d.dat", num);
//  if ((fp = fopen(name, "r")) == NULL) {
//    return d;
//  }
//  // 各部分領域間の最短距離を計算
//  d = Mat::zeros(subregion_num, subregion_num, CV_32FC1);
//  for (int y = 0; fgets(readline, MAX_LABEL, fp) != NULL; y++) {
//    d.at<float>(y, 0) = atoi(strtok(readline, " "));
//    for (int x = 1; x < subregion_num; x++) {
//      d.at<float>(y, x) = atoi(strtok(NULL, " "));
//    }
//  }
//
//  return d;
//}
//
///**
// * @brief
// * @param[in] subregion_num 移動する部分領域の数
// * @return
// */
//vector<Mat> FileOperation::readMultipleDistanceDat(const int start, const int end, 
//  const int subregion_num)
//{
//  vector<Mat> distmat;
//  for (int i = start; i < end;i++) {
//    Mat d = readDistanceDat(i, subregion_num);
//    if (d.empty()) {
//      break;
//    }
//    distmat.push_back(d);
//  }
//
//  return distmat;
//}

///**
// * @brief 部分領域間の最近点対の画素を読み込み
// * @param[in] 対象フレーム番号
// */
//void FileOperation::readDistancePointDat(const int num, const int x, const int y,
//  vector<Point> &pt1, vector<Point> &pt2)
//{
//  FILE *fp ;
//  char name[80];
//  char readline[MAX_FEATURE] = {'\0'};
//
//  int p1 = min(x, y);
//  int p2 = max(x, y);
//
//  // 部分領域間の最短距離の座標ペアのファイルを読み込み
//  sprintf(name, "./sequences/distancepoint-%d-%d-%d.dat", num, p1, p2);
//  if ((fp = fopen(name, "r")) == NULL) {
//    cerr << "error cannot read " << name << endl;
//    return;
//  }
//
//  for (int i = 0; fgets(readline, MAX_FEATURE, fp) != NULL; i++) {
//    Point p;
//    p.x = atoi(strtok(readline, " "));
//    p.y = atoi(strtok(NULL, " "));
//    pt1.push_back(p);
//
//    p.x = atoi(strtok(NULL, " "));
//    p.y = atoi(strtok(NULL, " "));
//    pt2.push_back(p);
//  }
//
//  fclose(fp);
//}
#endif

/**
 * @brief 部分領域間の最近点対の読み込み
 * @param[in] num     フレーム番号
 * @param[in] x       領域系列のラベル
 * @param[in] y       領域系列のラベル
 * @param[out]  pt1   最近点対のリスト（フレームnumに最近点対を追加）
 * @param[out]  pt2   最近点対のリスト（フレームnumに最近点対を追加）
 * @param[in] start   先頭フレーム番号
 * @param[in] end     末尾フレーム番号
 */
void FileOperation::readDistancePointDat(const int num, const int x, const int y,
  vector<Point> &pt1, vector<Point> &pt2, const int start, const int end)
{
  FILE *fp ;
  char name[80];
  char readline[MAX_FEATURE] = {'\0'};

  int p1 = min(x, y);
  int p2 = max(x, y);

  // 部分領域間の最短距離の座標ペアのファイルを読み込み
  sprintf(name, "./%d-%d/distancepoint-%d-%d-%d.dat", start, end, num, p1, p2);
  if ((fp = fopen(name, "r")) == NULL) {
    cerr << "error cannot read " << name << endl;
    return;
  }

  for (int i = 0; fgets(readline, MAX_FEATURE, fp) != NULL; i++) {
    Point p;
    p.x = atoi(strtok(readline, " "));
    p.y = atoi(strtok(NULL, " "));
    pt1.push_back(p);

    p.x = atoi(strtok(NULL, " "));
    p.y = atoi(strtok(NULL, " "));
    pt2.push_back(p);
  }

  fclose(fp);
}