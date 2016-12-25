/* Kinect.cpp */
#include "KinectCalibration.h"

/**
* @brief コンストラクタ
*/
Kinect::Kinect(void) : kinect(0)
{
}

/**
* @briefデストラクタ
*/
Kinect::~Kinect(void)
{
  // Kinect終了処理
  if (kinect != 0) {
    kinect->NuiShutdown();  // Kinectインスタンスの動作停止
    kinect->Release();      // Kinectインスタンスの解放
  }
}

/**
* @brief 初期化関数
* @param[in] max_depth 深度の最大値
* @return true 成功，false 失敗
*/
bool Kinect::initialize(float max_depth)
{
  this->max_depth = max_depth;

  // 接続Kinect数の確認
  int num = 0;
  NuiGetSensorCount(&num);
  if (num == 0) {
    cerr << "error - cannot find Kinect" << endl;
    return false;
  }

  // Kinectインスタンスの生成
  NuiCreateSensorByIndex(0, &kinect);

  // Kinectの初期化（深度カメラ利用の設定）
  result = kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | 
    NUI_INITIALIZE_FLAG_USES_DEPTH);
  if (result != S_OK) {
    cerr <<"error - cannot initialize kinect" << endl;
    return false;
  }

  // カラーカメラの初期化（解像度を640x480に設定）
  result = kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,
    NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &h_image);
  if (result != S_OK) {
    cerr <<"error - cannot open image stream of depth sensor" << endl;
    return false;
  }

  // 深度カメラの初期化（解像度を640x480に設定）
  result = kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH,
    NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE,
    2, 0, &h_depth);
  if (result != S_OK) {
    cerr <<"error - cannot open image stream of depth sensor" << endl;
    return false;
  }

  // Nearモードの設定
  result = kinect->NuiImageStreamSetImageFrameFlags(h_depth,
    NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
  if (result != S_OK) {
    cerr <<"error - cannot set near mode" << endl;
    return false;
  }

  // 640x480解像度の画像サイズを取得
  NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);

  return true;
}

/**
*
*/
void Kinect::colorAndDepthImage()
{
  // ストリームイベントの生成
  streamEvent = CreateEvent(0, TRUE, FALSE, 0);
  kinect->NuiSetFrameEndEvent(streamEvent, 0);

  // 変数の初期化
  Mat color(height, width, CV_8UC3, Scalar(0));
  Mat depth(height, width, CV_8UC1, Scalar(0));
  Mat temp;

  const char *cname = "Color";
  const char *dname = "Depth";
  const char *rname = "Region";
  realpointsmm.resize(width * height);

  // マウスコールバック関数の設定
  namedWindow(cname);
  namedWindow(dname);
  namedWindow(rname);
  setMouseCallback(cname, &Kinect::onMouse, this);
  setMouseCallback(dname, &Kinect::onMouse, this);
  setMouseCallback(rname, &Kinect::onMouse, this);

  int frame_num = 0;
  float dist_ave_ave = 0;
  char *name = "average.csv";
  FILE *fp = fopen(name, "w");

  while(1) {
    frame_num++;

    // 更新の設定
    WaitForSingleObject(streamEvent, INFINITE);
    ResetEvent(streamEvent);

    // 深度画像の取得
    NUI_IMAGE_FRAME depth_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

    // 深度データの取得
    NUI_LOCKED_RECT depth_data = {0};
    depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );

    USHORT *depth_short = (USHORT*)depth_data.pBits;

    // 平板間の距離の平均を計算
    float dist_ave = 0;
    int cnt = 0;

    for (int y = 0; y < depth.rows; y++) {
      for (int x = 0; x < depth.cols; x++, depth_short++) {
        // 深度情報の取得
        Vector4 realpoints = NuiTransformDepthImageToSkeleton(x, y, 
          *depth_short, NUI_IMAGE_RESOLUTION_640x480);

        LONG depthX = x;
        LONG depthY = y;
        LONG colorX = depthX;
        LONG colorY = depthY;

        // 深度座標をカラー画像の座標へ補正する
        kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
          NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
          depthX , depthY, *depth_short, &colorX, &colorY);

        int index = (colorY * width) + colorX;

        // mからmmへ変換
        realpointsmm.at(index).x = realpoints.x * 1000; 
        realpointsmm.at(index).y = realpoints.y * 1000;
        realpointsmm.at(index).z = realpoints.z * 1000;

        // 8bitにして深度画像に格納
        if (realpointsmm.at(index).z != 0 && realpointsmm.at(index).z <= max_depth) {
          depth.at<uchar>(index) = realpointsmm.at(index).z / max_depth * 255;
          if (x >= 280 && x <= 360 && y >= 200 && y <= 280) {
            dist_ave += realpointsmm.at(index).z;
            cnt++;
          }
        }
        else {
          depth.at<uchar>(index) = 0;
        }
      }
    }
    
    if (frame_num % 100 == 0) {
      dist_ave_ave += dist_ave / cnt;

      fprintf(fp, "%f\n", dist_ave / cnt);
      printf("average (count) : %f (%d)\n", dist_ave / cnt, cnt);
    }

    // カラー画像の取得
    NUI_IMAGE_FRAME image_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_image, INFINITE, &image_frame);

    NUI_LOCKED_RECT color_data;
    image_frame.pFrameTexture->LockRect(0, &color_data, 0, 0);

    // フレームデータの解放
    kinect->NuiImageStreamReleaseFrame(h_image, &image_frame);

    temp = Mat(height, width, CV_8UC4, color_data.pBits);
    cvtColor(temp, color, CV_BGRA2GRAY/*BGR*/);

    // 二値化
    cv::threshold(color, color, 100, 255, CV_THRESH_BINARY);
    cvtColor(color, color, CV_GRAY2BGR);

    // 重心を計算するため領域分割
    GraphBased gb(300, 10, 0, 0, 0);
    Mat flow = Mat::zeros(height, width, CV_32FC3);
    SubRegion region = gb.segmentImage(color, depth, flow);

    // 重心位置の推定
    Mat output;
    int region_num = preprocess(label, region);
    output = drawSubRegion(label);
    calcCentroid(label, region_num);

    // フレームデータの解放 (release frame data)
    kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

    // 画像出力
    cv::imshow(cname, color);
    cv::imshow(dname, depth);
    cv::imshow(rname, output);

    int key = waitKey(0);
    if (key == 'q') {
      break;
    }

    if (frame_num % 1000 == 0) {
      break;
    }
  }

  fprintf(fp, "%f\n", dist_ave_ave / 10);
  printf("average average : %f\n", dist_ave_ave / 10);

  fclose(fp);
}

/**
* @brief マウスコールバック関数
*/
void Kinect::onMouse(int event, int x, int y, int flags, void *param)
{
  // 左クリックイベントが発生したかどうか
  if (event == CV_EVENT_LBUTTONDOWN) {
    reinterpret_cast<Kinect *>(param)->read3DCoordinates(x, y);
  }

  // 右クリックイベントが発生したかどうか
  if (event == CV_EVENT_RBUTTONDOWN) {
    reinterpret_cast<Kinect *>(param)->readCentroid3DCoordinates(x, y);
  }
}

/**
* @brief ３次元座標の読み取り
* @param[in] x
* @param[in] y
*/
void Kinect::read3DCoordinates(int x, int y)
{
  int index = y * width + x;
  printf("3d (%d, %d) : %f, %f, %f\n", x, y, realpointsmm.at(index).x,
    realpointsmm.at(index).y, realpointsmm.at(index).z);
}

void Kinect::readCentroid3DCoordinates(int x, int y)
{
  int i = label.at<int>(y, x);
  x = centroids.at(i).x;
  y = centroids.at(i).y;
  int index = y * width + x;
  printf("centroid (%d, %d) : %f, %f, %f\n", x, y, realpointsmm.at(index).x,
    realpointsmm.at(index).y, realpointsmm.at(index).z);
  printf("prev centroid     : %f, %f, %f\n", prevpoint.x,
    prevpoint.y, prevpoint.z);

  // 前の結果との差を計算
  printf("diff : %f, %f, %f\n", abs(realpointsmm.at(index).x - prevpoint.x),
    abs(realpointsmm.at(index).y - prevpoint.y), abs(realpointsmm.at(index).z - prevpoint.z));

  // 前のクリックの結果として格納
  prevpoint.x = realpointsmm.at(index).x;
  prevpoint.y = realpointsmm.at(index).y;
  prevpoint.z = realpointsmm.at(index).z;
}

/**
 * @brief 前処理（ラベルの取得と部分領域の境界を抽出）
 * @param[out] label   部分領域のラベル
 * @param[in] region  部分領域
 * @return  部分領域の数
 */
int Kinect::preprocess(Mat &label, SubRegion &region)
{
  int width = region.width;
  int height = region.height;

  label.release();
  label = Mat::ones(height, width, CV_32S) * -1;
 
  vector<int> tmp;
  for (int i = 0; i < width * height; i++) {
    tmp.push_back(-1);
  }

  int region_num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = region.find(y * width + x);

      if (tmp[comp] == -1) {
        tmp[comp] = region_num;
        region_num++;
      }

      // 部分領域のラベルを取得
      label.at<int>(Point(x, y)) = tmp[comp];
    }
  }

  return region_num;
}

/**
* @brief 分割結果の色付け
* @param[in] label  ラベル
* @return  出力結果
*/
Mat Kinect::drawSubRegion(Mat &label)
{
  int width = label.cols;
  int height = label.rows;

  // 領域分割の結果を色付け
  Mat output(height, width, CV_8UC3);

  vector<Vec3b> rgb(width * height);
  for (int i = 0; i < width * height; i++) {
    rgb[i] = Vec3b(rand() % 256, rand() % 256, rand() % 256);
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      output.at<Vec3b>(y, x) = rgb[label.at<int>(y, x)];
    }
  }

  return output;
}

/**
 * @brief 各領域の重心を計算
 */
void Kinect::calcCentroid(const Mat &label, const int region_num)
{
  // 重心を格納する配列のメモリを確保
  centroids.clear();
  centroids.resize(region_num);
  vector<int> size(region_num);

  // 重心の計算
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = label.at<int>(y, x);
      centroids.at(index).x += x;
      centroids.at(index).y += y;
      size.at(index)++;
    }
  }

  for (int i = 0; i < region_num; i++) {
    centroids.at(i).x /= size.at(i);
    centroids.at(i).y /= size.at(i);
  }
}