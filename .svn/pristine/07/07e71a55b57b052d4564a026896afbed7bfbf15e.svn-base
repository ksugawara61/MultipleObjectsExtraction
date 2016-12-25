/*****************************************************************************/
/*! @addtogroup 
 *  @file   Kinect.cpp
 *  @brief  Kinectの処理に関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "Kinect.h"

/**
 * @brief コンストラクタ
 */
Kinect::Kinect(void) : kinect(0)
{
}

/**
 * @brief デストラクタ
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
 * @brief Kinectの初期化
 * @param[in] max_depth 出力する深度情報の最大値（mm）
 * @return  true 成功，false 失敗
 */
bool Kinect::initialize(float max_depth)
{
  this->max_depth = max_depth;

  // 最大深度の取得
  FILE *fp;
  if((fp = fopen("./video/max_depth.csv", "w")) == NULL) {
    cerr << "error - cannnot open max_depth.csv" << endl;
    return false;
  }
  fprintf(fp,"%f\n", max_depth);
  fclose(fp);

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

  /////////////////////////////////////////////////////////////////////////////
  // 有効範囲外 0.8～4.0以外のデータを取得
  //kinect->NuiImageStreamSetImageFrameFlags(h_depth, NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES);
  /////////////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////
  // Nearモードの設定
  result = kinect->NuiImageStreamSetImageFrameFlags(h_depth,
    NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
  if (result != S_OK) {
    cerr <<"error - cannot set near mode" << endl;
    return false;
  }
  /////////////////////////////////////////////////////////////////////////////

  // 640x480解像度の画像サイズを取得
  NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);

  return true;
}

/**
 * @brief 画像情報取得関数
 */
bool Kinect::colorImage(Mat &image)
{
  // カラー画像の取得
  NUI_IMAGE_FRAME image_frame = {0};
  kinect->NuiImageStreamGetNextFrame(h_image, INFINITE, &image_frame);

  NUI_LOCKED_RECT color_data;
  image_frame.pFrameTexture->LockRect(0, &color_data, 0, 0);

  // フレームデータの解放
  kinect->NuiImageStreamReleaseFrame(h_image, &image_frame);

  Mat temp = Mat(height, width, CV_8UC4, color_data.pBits);
  cvtColor(temp, image, CV_BGRA2BGR);

  return true;
}

/**
 * @brief 深度画像取得関数
 * @param[out] img image
 *
 * @return true 正常終了，false 異常終了
 */
bool Kinect::depthImage(Mat &img)
{
  //userMask = cv::Mat( height, width, CV_8UC1, cv::Scalar( 0 ) );
  //image = cv::Mat( height, width, CV_8UC1, cv::Scalar ( 0 ) );

  img = Mat(height, width, CV_8UC1, Scalar(0));

  // 深度画像の取得
  NUI_IMAGE_FRAME depth_frame = {0};
  kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

  // 距離データの取得
  NUI_LOCKED_RECT depth_data = {0};
  depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );

  USHORT *depth = (USHORT*)depth_data.pBits;
  for (int i = 0; i < (depth_data.size / sizeof(USHORT)); i++) {
    // 深度情報の取得
    USHORT distance = ::NuiDepthPixelToDepth(depth[i]);
    //USHORT player = ::NuiDepthPixelToPlayerIndex( depth[i] );

    LONG depthX = i % width;
    LONG depthY = i / width;
    LONG colorX = depthX;
    LONG colorY = depthY;

    // 深度座標をカラー画像の座標へ変換
    kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
      NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
      depthX , depthY, 0, &colorX, &colorY);

    // ユーザピクセルかどうかをマスク画像に記録
    //if ( player != 0 ) {
    //  int index = (colorY * width) + colorX;
    //  userMask.data[index] = 255;
    //}

    // 8bitにして深度画像に格納
    int index = (colorY * width) + colorX;
    img.data[index] = distance / 8192.0 * 255;

    //cout << "(" << colorX << ", " << colorY << ") " << distance << endl;
  }

  // フレームデータの解放(release frame data)
  kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

  return true;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 *
 */
bool Kinect::depthSave(string name)
{
  FILE *fp;
  name += ".csv";

  // ファイルのオープン (open file)
  if ((fp = fopen(name.c_str(), "w")) == NULL) {
    cerr << "cannnot open " << name << " file" << endl;
    return false;
  }

  // 深度画像の取得
  NUI_IMAGE_FRAME depth_frame = {0};
  kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

  // 距離データの取得
  NUI_LOCKED_RECT depth_data = {0};
  depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );

  USHORT *depth = (USHORT*)depth_data.pBits;
  for (int i = 0; i < (depth_data.size / sizeof(USHORT)); i++) {
    // 深度情報の取得
    USHORT distance = ::NuiDepthPixelToDepth(depth[i]);

    LONG depthX = i % width;
    LONG depthY = i / width;
    LONG colorX = depthX;
    LONG colorY = depthY;

    // 深度座標をカラー画像の座標へ変換
    kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
      NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
      depthX , depthY, 0, &colorX, &colorY);

    fprintf(fp, "%d, %d, %d\n", colorX, colorY, distance);
    //content << colorX << ", " << colorY << ", " << distance << endl;
  }

  //string temp = content.str();
  //fprintf(fp, "%s", temp.c_str());

  // フレームデータの解放 (release frame data)
  kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

  // ファイルのクローズ (close file);
  fclose(fp);

  return true;
}

/**
* 機能：XZ画像出力関数
* パラメータで出力する距離範囲を設定するほうが確実な
*/
bool Kinect::depthImageXZ(Mat &img)
{
  img = Mat(height, width, CV_8UC1, Scalar(0));

  // 深度画像の取得
  NUI_IMAGE_FRAME depth_frame = {0};
  kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

  // 距離データの取得
  NUI_LOCKED_RECT depth_data = {0};
  depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );

  USHORT *depth = (USHORT*)depth_data.pBits;

  // 画像の初期化
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      img.data[(y * width) + x] = 0;
    }
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // 深度情報の取得
      USHORT distance = NuiDepthPixelToDepth(depth[(y * width) + x]);

      LONG depthX = x;
      LONG depthY = y;
      LONG colorX = depthX;
      LONG colorY = depthY;

      // 深度座標をカラー画像の座標へ変換
      //kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
      //  NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
      //  depthX , depthY, 0, &colorX, &colorY);

      if (distance != 0) {
        // 最大8192.0だが取得する範囲から2000に設定
        //USHORT index = (width * distance / 2000.0) * width + colorX /*8192.0*/;
        USHORT index = height * distance / 3000.0;
        // 現在の処理ではy軸の低い位置にものがあるほど色が白くなる
        img.data[index * width + colorX] = 255 * colorY / height;
      }
    }
  }

  // フレームデータの解放(release frame data)
  kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

  return true;
}


/**
* 機能：YZ画像出力関数
* パラメータで出力する距離範囲を設定するほうが確実な
*/
bool Kinect::depthImageYZ(Mat &img)
{
  img = Mat(height, width, CV_8UC1, Scalar(0));

  // 深度画像の取得
  NUI_IMAGE_FRAME depth_frame = {0};
  kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

  // 距離データの取得
  NUI_LOCKED_RECT depth_data = {0};
  depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );

  USHORT *depth = (USHORT*)depth_data.pBits;

  // 画像の初期化
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      img.data[(y * width) + x] = 0;
    }
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // 深度情報の取得
      USHORT distance = NuiDepthPixelToDepth(depth[(y * width) + x]);

      LONG depthX = x;
      LONG depthY = y;
      LONG colorX = depthX;
      LONG colorY = depthY;

      // 深度座標をカラー画像の座標へ変換
      kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
        NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
        depthX , depthY, 0, &colorX, &colorY);

      if (distance != 0) {
        // 最大8192.0だが取得する範囲から2000に設定
        USHORT index = width * distance / 3000.0 /*8192.0*/;
        img.data[(colorY * width) + index] = 255 * colorX / width;
      }
    }
  }

  // フレームデータの解放(release frame data)
  kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

  return true;
}
#endif

/**
 * @brief 画像データ生成関数
 * @param[in] fps 保存する動画のフレームレート
 */
void Kinect::createImageData(float fps)
{
  int cnt = 0;  // 30フレームを取得するまでの時間計測用
  clock_t s_time, e_time; // 処理時間計測用
  int frame_num = 0, key;  // フレーム番号
  Mat color, depth;

  // 保存するaviファイルの設定
  VideoWriter cwriter("./video/color.avi", CV_FOURCC('D','I','V','3'), fps, Size(640, 480), 1); // mpeg1 CV_FOURCC('P', 'I', 'M' '1'), 非圧縮CV_FOURCC('D', 'I', 'B', ' ')
  VideoWriter dwriter("./video/depth.avi", CV_FOURCC('D','I','V','3'), fps, Size(640, 480), 0); // CV_FOURCC('D','I','B',' ')

  // ストリームイベントの生成
  streamEvent = CreateEvent(0, TRUE, FALSE, 0);
  kinect->NuiSetFrameEndEvent(streamEvent, 0);

  s_time = clock();
  while(1) {
    if (cnt == 30) {
      cnt = 0;
      e_time = clock();
      cout << "30 frame : " << e_time - s_time << "[ms]" << endl;
      s_time = e_time;
    }
    else {
      cnt++;
    }

    // Matの初期化
    color = Mat::zeros(height, width, CV_8UC3);
    depth = Mat::zeros(height, width, CV_8UC1);

    // 更新の設定
    WaitForSingleObject(streamEvent, INFINITE);
    ResetEvent(streamEvent);

    // 画像情報の取得
    NUI_IMAGE_FRAME image_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_image, INFINITE, &image_frame);

    NUI_LOCKED_RECT color_data;
    image_frame.pFrameTexture->LockRect(0, &color_data, 0, 0);

    // 画像情報をMatに格納し，RGBAからBGRに変換
    color = Mat(height, width, CV_8UC4, color_data.pBits);
    cvtColor(color, color, CV_BGRA2BGR);

    // 深度情報の取得
    NUI_IMAGE_FRAME depth_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

    NUI_LOCKED_RECT depth_data = {0};
    depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );
    USHORT *depth_short = (USHORT*)depth_data.pBits;
    
    for (int i = 0; i < (depth_data.size / sizeof(USHORT)); i++) {
      // 注目画素の深度情報の取得
      USHORT distance = ::NuiDepthPixelToDepth(depth_short[i]);

      LONG depthX = i % width;
      LONG depthY = i / width;
      LONG colorX = depthX;
      LONG colorY = depthY;

      // 深度座標をカラー画像の座標へ変換
      kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
        NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
        depthX , depthY, depth_short[i], &colorX, &colorY);

      // 8bitにして深度画像に格納
      int index = (colorY * width) + colorX;
      if (distance <= max_depth && distance != 0) {
        depth.at<uchar>(index) = distance / max_depth * 255;
      }
      // 獲得する深度の最大値を超える場合，欠損値として扱う
      else {
        depth.at<uchar>(index) = 0;
      }
    }

    // フレームの格納
    cwriter << color;
    dwriter << depth;

    // フレームデータの解放 (release frame data)
    kinect->NuiImageStreamReleaseFrame(h_image, &image_frame);
    kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

    // 画像の反転
    flip(color, color, 1);
    flip(depth, depth, 1);

    // 画像出力
    imshow("Color", color);
    imshow("Depth", depth);

    frame_num++;

    int key = waitKey(1);
    if (key == 'q') {
      break;
    }
  }

  cwriter.release();
  dwriter.release();
}


/**
 * @brief 正解データ用の画像・深度情報の生成
 * @param[in] fps 保存する動画のフレームレート
 */
void Kinect::createCorrectImageData(float fps)
{
  int cnt = 0;  // 30フレームを取得するまでの時間計測用
  clock_t s_time, e_time; // 処理時間計測用
  int frame_num = 0, key;  // フレーム番号
  Mat color, depth;

  // 保存するaviファイルの設定
  VideoWriter cwriter("./video/color.avi", CV_FOURCC('D','I','V','3'), fps, Size(640, 480), 1); // mpeg1 CV_FOURCC('P', 'I', 'M' '1'), 非圧縮CV_FOURCC('D', 'I', 'B', ' ')
  VideoWriter dwriter("./video/depth.avi", CV_FOURCC('D','I','V','3'), fps, Size(640, 480), 0); // CV_FOURCC('D','I','B',' ')

  // ストリームイベントの生成
  streamEvent = CreateEvent(0, TRUE, FALSE, 0);
  kinect->NuiSetFrameEndEvent(streamEvent, 0);
  
  s_time = clock();
  while(1) {
    if (cnt == 30) {
      cnt = 0;
      e_time = clock();
      cout << "30 frame : " << e_time - s_time << "[ms]" << endl;
      s_time = e_time;
    }
    else {
      cnt++;
    }

    // Matの初期化
    color = Mat::zeros(height, width, CV_8UC3);
    depth = Mat::zeros(height, width, CV_8UC1);

    // 更新の設定
    WaitForSingleObject(streamEvent, INFINITE);
    ResetEvent(streamEvent);

    // 画像情報の取得
    NUI_IMAGE_FRAME image_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_image, INFINITE, &image_frame);

    NUI_LOCKED_RECT color_data;
    image_frame.pFrameTexture->LockRect(0, &color_data, 0, 0);
    BYTE *color_short = (BYTE*)color_data.pBits;

    // 画像情報をMatに格納し，RGBAからBGRに変換
    //color = Mat(height, width, CV_8UC4, color_data.pBits);
    //cvtColor(color, color, CV_BGRA2BGR);

    // 深度情報の取得
    NUI_IMAGE_FRAME depth_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

    NUI_LOCKED_RECT depth_data = {0};
    depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );

    USHORT *depth_short = (USHORT*)depth_data.pBits;
    for (int i = 0; i < (depth_data.size / sizeof(USHORT)); i++) {
      // 注目画素の深度情報の取得
      USHORT distance = ::NuiDepthPixelToDepth(depth_short[i]);

      LONG depthX = i % width;
      LONG depthY = i / width;
      LONG colorX = depthX;
      LONG colorY = depthY;

      // 深度座標をカラー画像の座標へ変換
      kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
        NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
        depthX , depthY, depth_short[i], &colorX, &colorY);

      // 8bitにして深度画像に格納
      int index = (colorY * width) + colorX;
      if (distance <= max_depth && distance != 0) {
        depth.at<uchar>(index) = distance / max_depth * 255;
        int tmp = index * 4;
        color.at<Vec3b>(index) = 
          Vec3b(color_short[tmp], color_short[tmp + 1], color_short[tmp + 2]);
      }
      // 獲得する深度の最大値を超える場合，欠損値として扱う
      else {
        depth.at<uchar>(index) = 0;
        color.at<Vec3b>(index) = Vec3b(0, 0, 0);
      }
    }

    // 対象フレームの保存
    cwriter << color;
    dwriter << depth;

    // フレームデータの解放
    kinect->NuiImageStreamReleaseFrame(h_image, &image_frame);
    kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

    // 画像・深度情報の反転
    flip(color, color, 1);
    flip(depth, depth, 1);

    // 画像・深度情報の出力
    imshow("Color", color);
    imshow("Depth", depth);

    frame_num++;

    key = waitKey(1);
    if (key == 'q') {
      break;
    }
  }

  cwriter.release();
  dwriter.release();
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 * 深度画像生成画像
 */
void Kinect::createDepthImage(char *name)
{
  Mat depth(height, width, CV_8UC1, Scalar(0));

  // ファイルのオープン (open file)
  FILE *fp;
  if ((fp = fopen(name, "r")) == NULL) {
    cerr << "cannnot open " << name << " file" << endl;
    return;
  }

  // csvファイルの読み込み
  char buf[20], *tmp;
  CvPoint3D64f pt;
  vector<CvPoint3D64f> v_pt;
  for (int i = 0; fgets(buf, 20, fp) != NULL; i++) {
    tmp = strtok(buf, ",");
    pt.x = atof(tmp);
    tmp = strtok(NULL, ",");
    pt.y = atof(tmp);
    tmp = strtok(NULL, "\n");
    pt.z = atof(tmp);

    v_pt.push_back(pt);
  }

  // 深度画像の生成
  int size = v_pt.size();
  for (int i = 0; i < size; i++) {
    double distance = v_pt.at(i).z;
    int x = v_pt.at(i).x;
    int y = v_pt.at(i).y;

    depth.data[(y * width) + x] = distance / 8192.0 * 255;
  }

  fclose(fp);

  // 画像表示
  cv::imshow("Depth", depth);

  int key = cv::waitKey(0);
}

/**
 * 文字列変換関数
 */
string Kinect::int2String(int num)
{
  stringstream ss;

  ss << num;

  return ss.str();
}

#endif