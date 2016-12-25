/*****************************************************************************/
/*! @addtogroup 
 *  @file   FileOperationWrite.cpp
 *  @brief  ファイル操作（書き込み）に関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/
#include "FileOperation.h"

/**
 * @brief 部分領域のラベルのファイル書き込み
 * @param[in] label 部分領域のラベル
 * @param[in] num   フレーム番号
 */
void FileOperation::writeLabelDat(const Mat &label, const int num)
{
  int width = label.cols;
  int height = label.rows;

  _mkdir("./subregion");

  char name[80];
  sprintf(name, "./subregion/label-0%004d.dat", num);
  FILE *fp = fopen(name, "w");

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      std::fprintf(fp, "%d ", label.at<int>(y, x));
    }
    std::fprintf(fp, "\n");
  }

  fclose(fp);
}

/**
 * @brief 部分領域の特徴量のファイル書き込み
 * @param[in] features  部分領域の特徴量
 * @param[in] num       フレーム番号
 */
void FileOperation::writeFeaturesDat(const Features &features, const int num)
{
  int size = features.size();
  
  _mkdir("./subregion");

  char name[80];
  sprintf(name, "./subregion/feature-0%004d.dat", num);
  FILE *fp = fopen(name, "w");

  for (int i = 0; i < size; i++) {
    Feature f = features.at(i);
    std::fprintf(fp, "%d %d %f %f %f %f %f ",
      f.num, f.label, f.centroid.x, f.centroid.y,
      f.centroid3d.x, f.centroid3d.y, f.centroid3d.z);
    std::fprintf(fp, "%f %f %f %f %f %f ",
      f.flow2d.x, f.flow2d.y, f.flow2d.z, f.flow3d.x, f.flow3d.y, f.flow3d.z);
    std::fprintf(fp, "%f %f %f %f %f %f %f\n", 
      f.r, f.g, f.b, f.d, f.norm2d, f.norm3d, f.size);
  }

  fclose(fp);
}

/**
 * @brief シーンフローのファイル書き込み
 * @param[in] flow  シーンフロー
 * @param[in] num   フレーム番号
 */
void FileOperation::writeSceneFlowDat(const Mat &flow, const int num)
{
  int width = flow.cols;
  int height = flow.rows;
  char name[80];
  FILE *fp;

  _mkdir("./subregion");

  sprintf(name, "./subregion/sceneflow-0%004d.dat", num);
  fp = fopen(name, "w");

  // シーンフローをファイルに書き込み
  Point3f delta;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      delta = flow.at<Point3f>(y, x);
      std::fprintf(fp, "%d %d %.5f %.5f %.5f\n", x, y, delta.x, delta.y, delta.z);
    }
  }

  fclose(fp);
}

//vector<FeatureSequence> FileOperation::writeSubRegionSequencesDat(TemporalSubRegion &region, 
//  vector<Features> &f_vec, vector<Mat> &labels, const int N,
//  const int start, const int end, const int min_frame, const float min_move, bool flag)
//{
//  char name[80];
//  sprintf(name, "./%d-%d", start, end);
//  _mkdir(name);
//
//  vector<FeatureSequence> sequences;
//
//  // 部分領域系列の計算
//  int frame, label, r_next, r_prev;
//  int frame_num = f_vec.size();
//  int region_num;
//  for (int i = 0; i < frame_num; i++) {
//    region_num = f_vec.at(i).size();
//    for (int j = 0; j < region_num; j++) {
//      region.find(i, j, frame, label, r_next, r_prev);
//
//      // 対応付けされているかを取得
//      if ((r_next != 0 || r_prev != 0) && region.getNorm(frame, label)) {          
//        bool flag = false;
//
//        // 部分領域系列へ追加
//        int size = sequences.size();
//        for (int k = 0; k < size; k++) {
//          if (sequences.at(k).getFrame() == frame &&
//            sequences.at(k).getLabel() == label) 
//          {
//
//            sequences.at(k).assign(i, f_vec.at(i).at(j));
//            flag = true;
//            break;
//          }
//        }
//
//        // 新しい部分領域系列を作成
//        if (!flag) {
//          FeatureSequence s(frame, label, frame_num);
//          s.assign(i, f_vec.at(i).at(j));
//          sequences.push_back(s);
//        }
//      }
//    }
//  }
//
//  vector<int> original;
//  int sequences_num = sequences.size();
//
//  // 系列の長さを計算
//  for (int i = 0; i < sequences_num; i++) {
//    int length = 0; // 系列の長さ
//    float moving = 0; // 系列の総移動量
//    for (int j = 0; j < frame_num; j++) {
//      if (sequences.at(i).getLabels(j).size() != 0) {
//        length++;
//        moving += sequences.at(i).getFeature(j).norm3d;
//      }
//    }
//    moving /= length;
//
//    // 短い系列，移動の小さい系列の除去
//    if (length < min_frame || moving < min_move) {
//      sequences.erase(sequences.begin() + i);
//      sequences_num--;
//      i--;
//      continue;
//    }
//
//    // 系列の長さを代入
//    sequences.at(i).setLength(length);
//  }
//
//  FILE *fp;
//  for (int i = 0; i < sequences_num; i++) {
//    // 系列全体の特徴を書き込み
//    sprintf(name, "./%d-%d/subregion-%d.dat", start, end, i);
//    fp = fopen(name, "w");
//    fprintf(fp, "%d %d %d\n", 
//      sequences.at(i).getFrame(), sequences.at(i).getLabel(),
//      end - start + 1);
//
//    // 各系列の特徴を書き込み
//    for (int j = 0; j < frame_num; j++) {
//      sequences.at(i).average(j); // 平均の計算
//      Feature &pf = sequences.at(i).getFeature(j);
//      fprintf(fp, "%d %f %f %f %f %f ",
//        j, pf.centroid.x, pf.centroid.y,
//        pf.centroid3d.x, pf.centroid3d.y, pf.centroid3d.z);
//      fprintf(fp, "%f %f %f %f %f %f ",
//        pf.flow2d.x, pf.flow2d.y, pf.flow2d.z,
//        pf.flow3d.x, pf.flow3d.y, pf.flow3d.z);
//
//      original = sequences.at(i).getLabels(j);
//      fprintf(fp, "%f %f %f %f %f %f %f %d ", 
//        pf.r, pf.g, pf.b, pf.d, pf.norm2d, pf.norm3d, pf.size,
//        original.size());
//
//      // 対応付けられているもとのラベルを書き込む
//      if (original.size() != 0) {
//        for (int k = 0; k < original.size(); k++) {
//          fprintf(fp, "%d ", original.at(k));
//        }
//      }
//      fprintf(fp, "\n");
//    }
//
//    // 系列の長さを記述
//    fprintf(fp, "%d\n", sequences.at(i).getLength());
//
//    fclose(fp);
//  }
//
//  // ここから距離の計算
//  clock_t s_time = clock(), e_time; // 処理時間計測用
//  /*for (int i = 0; i < frame_num; i++) {
//    writeDistanceDat(region, labels.at(i), sequences, i, start, 3);
//  }
//  e_time = clock();*/
//
//  // ここから距離の計算をスレッドで行う
//  //（GPUを用いて，並列処理が出来ればかなり処理速度が速くなると思う）
//  if (flag) {
//    vector<WriteNPThread> thread;
//
//    int i = 0;
//    while(1) {
//
//      // スレッドクラスのメモリ確保
//      int cnt = 0;
//      for (int j = 0; i < frame_num && j < 6; i++, cnt++, j++) {
//        thread.push_back(*(new WriteNPThread(region, sequences, labels.at(i), i, start, end)));
//      }
//
//      // スレッドの生成
//      for (int j = 0; j < 6 && j < cnt; j++) {
//        thread.at(j).createThread(N);
//      }
//
//      // スレッドのメモリ解放（スレッド終了まで待機）
//      thread.clear();
//
//      if (i >= frame_num) {
//        break;
//      }
//    }
//
//    e_time = clock();
//
//    cout << "nearest point calculation time : " << e_time - s_time << "[ms]" << endl;
//  }
//
//  return sequences;
//}

/**
 * @brief 領域系列の特徴量のファイル書き込み
 * @param[in] region_seq  領域系列のリスト
 * @param[in] start       先頭フレーム番号
 * @param[in] end         末尾フレーム番号
 */
void FileOperation::writeRegionSequencesDat(RegionSequences &region_seq,
  const int start, const int end)
{
  // 変数宣言
  FILE *fp;
  char name[80];
  int seq_num = region_seq.size();
  int frame_num = end - start + 1;

  sprintf(name, "./%d-%d", start, end);
  _mkdir(name);

  // 領域系列の特徴量をファイル出力
  for (int i = 0; i < seq_num; i++) {
    // 領域系列全体の特徴量を書き込み
    sprintf(name, "./%d-%d/subregion-%d.dat", start, end, i);
    fp = fopen(name, "w");
    RegionSequence &region = region_seq.at(i);
    fprintf(fp, "%d %d %d\n", region.getLabel().frame, 
      region.getLabel().label, region.getLength());

    // 各領域系列の特徴量を書き込み
    for (int j = 0; j < frame_num; j++) {
      Features &pfeatures = region.getFeatures(j);

      for (int k = 0; k < pfeatures.size(); k++) {
        Feature &pf = pfeatures.at(k);

        fprintf(fp, "%d %d %f %f %f %f %f ",
          j, pf.label, pf.centroid.x, pf.centroid.y,
          pf.centroid3d.x, pf.centroid3d.y, pf.centroid3d.z);
        fprintf(fp, "%f %f %f %f %f %f ",
          pf.flow2d.x, pf.flow2d.y, pf.flow2d.z,
          pf.flow3d.x, pf.flow3d.y, pf.flow3d.z);

        fprintf(fp, "%f %f %f %f %f %f %f\n", 
          pf.r, pf.g, pf.b, pf.d, pf.norm2d, pf.norm3d, pf.size);
      }
    }

    fclose(fp);
  }
}

/**
 * @brief 最近点対の計算と書き込み（かなり処理が遅い）
 * @param[in] region_seq  領域系列のリスト
 * @param[in] labels      部分領域のラベルのリスト
 * @param[in] N           最近点対の数
 * @param[in] start       先頭フレーム番号
 * @param[in] end         末尾フレーム番号
 */
void FileOperation::writeNearestPointDat(RegionSequences &region_seq, vector<Mat> &labels, const int N,
  const int start, const int end)
{
  // 変数宣言
  int frame_num = end - start + 1;
  vector<WriteNearestThread> thread;
  clock_t s_time = clock(), e_time; // 処理時間計測用
  char name[80];

  sprintf(name, "./%d-%d", start, end);
  _mkdir(name);

  // ここから距離の計算をスレッドで行う
  //（GPUを用いて，並列処理が出来ればかなり処理速度が速くなると思う）
  int i = 0;
  while(1) {

    // スレッドクラスのメモリ確保
    int cnt = 0;
    for (int j = 0; i < frame_num && j < 6; i++, cnt++, j++) {
      thread.push_back(*(new WriteNearestThread(getFrameFeatures(region_seq, i), 
        region_seq.size(), labels.at(i), i, start, end)));
    }

    // スレッドの生成
    for (int j = 0; j < 6 && j < cnt; j++) {
      thread.at(j).createThread(N);
    }

    // スレッドのメモリ解放（スレッド終了まで待機）
    thread.clear();

    if (i >= frame_num) {
      break;
    }
  }

  e_time = clock();

  cout << "nearest point calculation time : " << e_time - s_time << "[ms]" << endl;
}