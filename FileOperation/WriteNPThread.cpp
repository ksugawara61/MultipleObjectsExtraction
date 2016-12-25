/*****************************************************************************/
/*! @addtogroup 
 *  @file   WriteNPThread.cpp
 *  @brief  最近点対の計算・ファイル書き込みに関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "WriteNPThread.h"

/**
 * @brief コンストラクタ
 * @param[in] f       部分領域の特徴量
 * @param[in] seq_num 領域系列の数
 * @param[in] label   部分領域のラベル
 * @param[in] t       フレーム番号
 * @param[in] start   先頭フレーム番号
 * @param[in] end     末尾フレーム番号
 */
WriteNearestThread::WriteNearestThread(Features f, int seq_num, Mat label,  
  int t, int start, int end)
{
//  this->region = region;
//  this->region_seq = region_seq;
  this->f = f;
  this->seq_num = seq_num;
  this->label = label.clone();
  this->t = t;
  this->start = start;
  this->end = end;

  h_thread = NULL;

  flag = false;
}

/**
 * @brief デストラクタ
 */
WriteNearestThread::~WriteNearestThread(void)
{
  // スレッド終了を待機
  WaitForSingleObject(h_thread, INFINITE);

  // ハンドラのクローズ
  CloseHandle(h_thread);
}

/**
 * @brief スレッドの開始
 * @param[in] N 最近点対の数
 */
void WriteNearestThread::createThread(int N)
{
  this->N = N;

  // 既にスレッドが生成されているか
  if (h_thread != NULL) {
    cerr <<"error - thread has been running" << endl;
    return;
  }

  // スレッドの生成
  h_thread = (HANDLE)_beginthreadex(0, 0, &WriteNearestThread::run, this, 0, 0);

  // スレッド開始まで待機
  //while(!flag);
}

/**
 * 
 */
unsigned __stdcall WriteNearestThread::run(void *param)
{
  // 動的なメンバ関数の呼び出し
//  reinterpret_cast<WriteNearestThread *>(param)->writeNearestPointDat();
  reinterpret_cast<WriteNearestThread *>(param)->writeNearestPointDatUsingBoundary();

  return 0;
}

/**
 * @brief 最近点対の計算・ファイル書き込み
 */
void WriteNearestThread::writeNearestPointDat(void)
{
  // スレッドの開始
  flag = true;

  char name[80];
  FILE *fp;
  int width = label.cols;
  int height = label.rows;
  int l;
  
  // 距離の計算用
  vector<Point> pt1(N), pt2(N);
  vector<Point3f> pt1_3d(N), pt2_3d(N);

  vector<float> dist(N);
  vector<vector<Point>> min_point(seq_num);
  vector<vector<Point3f>> min_point3d(seq_num);
  float d;
 
  sprintf(name, "./data/depth-0%004d.bmp", t + start);
  depth = imread(name, CV_LOAD_IMAGE_GRAYSCALE);

  // 描画候補の部分領域を昇順にソート
  sort(f.begin(), f.end(), featuresLabelAsc);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      l = label.at<int>(y, x);
      
      // 対応する領域系列を二分探索
      int low = 0, middle = 0, high = f.size() - 1;
      while(low <= high) {
        middle = (low + high) / 2;

        if (f.at(middle).label == l) {
          // 領域系列の画素の挿入
          min_point.at(f.at(middle).index).push_back(Point(x, y));

          d = depth.at<uchar>(y, x);
          min_point3d.at(f.at(middle).index).push_back(
            depth2World(x, y, luminance2Depth(d), width, height));

          break;
        }
        else if (f.at(middle).label < l) {
          low = middle + 1;
        }
        else {
          high = middle - 1;
        }
      }
    }
  }

  // 各部分領域間の最短距離を計算
  for (int y = 0; y < seq_num; y++) {
    for (int x = y + 1; x < seq_num; x++) {
      vector<Point> &bpY = min_point.at(y);
      vector<Point> &bpX = min_point.at(x);

      vector<Point3f> &bpY3d = min_point3d.at(y);
      vector<Point3f> &bpX3d = min_point3d.at(x);

      // 初期化
//      float min = FLT_MAX;
      for (int k = 0; k < N; k++) {
        pt1[k] = Point(0, 0);
        pt2[k] = Point(0, 0);
        dist[k] = FLT_MAX;

        pt1_3d[k] = Point3f(0, 0, 0);
        pt2_3d[k] = Point3f(0, 0, 0);
      }

      // ３次元位置の比較
      for (int i = 0; i < bpY3d.size(); i++) {
        Point &comp1 = bpY.at(i);          
        Point3f &comp1_3d = bpY3d.at(i);

        for (int j = 0; j < bpX3d.size(); j++) {
          Point &comp2 = bpX.at(j);
          Point3f &comp2_3d = bpX3d.at(j);
          float comp3d = distanceCentroid3d(comp1_3d, comp2_3d);

          for (int k = 0; k < N; k++) {
            if (dist[k] > comp3d && comp3d != 0.0) {
              // 最短距離の画素の順番をずらす
              for (int l = N - 1; l > k; l--) {
                dist[l] = dist[l - 1];
                pt1[l] = pt1[l - 1];
                pt2[l] = pt2[l - 1];
                pt1_3d[l] = pt1_3d[l - 1];
                pt2_3d[l] = pt2_3d[l - 1];
              }

              dist[k] = comp3d;

              pt1[k] = comp1;
              pt2[k] = comp2;

              pt1_3d[k] = comp1_3d;
              pt2_3d[k] = comp2_3d;

              break;
            }
          }
        }
      }

      // 二つの部分領域間の最短距離の座標ペアをファイル出力
      sprintf(name, "./%d-%d/distancepoint-%d-%d-%d.dat", start, end,
        t + start, y, x);
      fp = fopen(name, "w");

      // 全ての画素対を書き込み
      for (int i = 0; i < N; i++) {
        fprintf(fp, "%d %d %d %d\n", 
          pt1[i].x, pt1[i].y, pt2[i].x, pt2[i].y);
      }

      fclose(fp);
    }
  }

  cout << "thread " << t << " terminated" << endl;
}

/**
 * @brief 最近点対の書き込み（部分領域の境界を最近点対の計算に利用）
 */
void WriteNearestThread::writeNearestPointDatUsingBoundary(void)
{
  // スレッドの開始
  flag = true;

  char name[80];
  FILE *fp;
  int width = label.cols;
  int height = label.rows;
  int l;
  
  // 距離の計算用
  vector<Point> pt1(N), pt2(N);
  vector<Point3f> pt1_3d(N), pt2_3d(N);

  vector<float> dist(N);
  vector<vector<Point>> min_point(seq_num);
  vector<vector<Point3f>> min_point3d(seq_num);
  float d;
 
  sprintf(name, "./data/depth-0%004d.bmp", t + start);
  depth = imread(name, CV_LOAD_IMAGE_GRAYSCALE);

  // 部分領域間の境界
  sprintf(name, "./subregion/boundary-0%004d.png", t + start);
  boundary = imread(name, CV_LOAD_IMAGE_GRAYSCALE);

  if (boundary.empty()) {
    cout << "error - " << name << endl;
    return;
  }

  // 描画候補の部分領域を昇順にソート
  sort(f.begin(), f.end(), featuresLabelAsc);

  //// 境界の計算
  //boundary = Mat::zeros(height, width, CV_8UC1);
  //for (int y = 0; y < height; y++) {
  //  for (int x = 0; x < width; x++) {
  //    int comp = label.at<int>(y, x);

  //    for (int yy = y - 5; yy < y + 5 && yy < height; yy++) {
  //      for (int xx = x - 5; xx < x + 5 && xx < width; xx++) {
  //        if (yy < 0 || xx < 0) continue;

  //        int comp2 = label.at<int>(yy, xx);
  //        if (comp != comp2) {
  //          // 部分領域の境界の描画
  //          line(boundary, Point(x, y), Point(x, y), 255);
  //        }
  //      }
  //    }
  //  }
  //}

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      l = label.at<int>(y, x);

      // 部分領域間の境界以外は無視
      if (boundary.at<uchar>(y, x) != 255) continue;
      
      // 対応する領域系列を二分探索
      int low = 0, middle = 0, high = f.size() - 1;
      while(low <= high) {
        middle = (low + high) / 2;

        if (f.at(middle).label == l) {
          // 領域系列の画素の挿入
          min_point.at(f.at(middle).index).push_back(Point(x, y));

          d = depth.at<uchar>(y, x);
          min_point3d.at(f.at(middle).index).push_back(
            depth2World(x, y, luminance2Depth(d), width, height));

          break;
        }
        else if (f.at(middle).label < l) {
          low = middle + 1;
        }
        else {
          high = middle - 1;
        }
      }
    }
  }

  // 各部分領域間の最短距離を計算
  for (int y = 0; y < seq_num; y++) {
    for (int x = y + 1; x < seq_num; x++) {
      vector<Point> &bpY = min_point.at(y);
      vector<Point> &bpX = min_point.at(x);

      vector<Point3f> &bpY3d = min_point3d.at(y);
      vector<Point3f> &bpX3d = min_point3d.at(x);

      // 初期化
//      float min = FLT_MAX;
      for (int k = 0; k < N; k++) {
        pt1[k] = Point(0, 0);
        pt2[k] = Point(0, 0);
        dist[k] = FLT_MAX;

        pt1_3d[k] = Point3f(0, 0, 0);
        pt2_3d[k] = Point3f(0, 0, 0);
      }

      // ３次元位置の比較
      for (int i = 0; i < bpY3d.size(); i++) {
        Point &comp1 = bpY.at(i);          
        Point3f &comp1_3d = bpY3d.at(i);

        for (int j = 0; j < bpX3d.size(); j++) {
          Point &comp2 = bpX.at(j);
          Point3f &comp2_3d = bpX3d.at(j);
          float comp3d = distanceCentroid3d(comp1_3d, comp2_3d);

          for (int k = 0; k < N; k++) {
            if (dist[k] > comp3d && comp3d != 0.0) {
              // 最短距離の画素の順番をずらす
              for (int l = N - 1; l > k; l--) {
                dist[l] = dist[l - 1];
                pt1[l] = pt1[l - 1];
                pt2[l] = pt2[l - 1];
                pt1_3d[l] = pt1_3d[l - 1];
                pt2_3d[l] = pt2_3d[l - 1];
              }

              dist[k] = comp3d;

              pt1[k] = comp1;
              pt2[k] = comp2;

              pt1_3d[k] = comp1_3d;
              pt2_3d[k] = comp2_3d;

              break;
            }
          }
        }
      }

      // 二つの部分領域間の最短距離の座標ペアをファイル出力
      sprintf(name, "./%d-%d/distancepoint-%d-%d-%d.dat", start, end,
        t + start, y, x);
      fp = fopen(name, "w");

      // 全ての画素対を書き込み
      for (int i = 0; i < N; i++) {
        fprintf(fp, "%d %d %d %d\n", 
          pt1[i].x, pt1[i].y, pt2[i].x, pt2[i].y);
      }

      fclose(fp);
    }
  }

  cout << "thread " << t << " terminated" << endl;
}