/* Kinect.cpp */
#include "KinectCalibration.h"

/**
* @brief �R���X�g���N�^
*/
Kinect::Kinect(void) : kinect(0)
{
}

/**
* @brief�f�X�g���N�^
*/
Kinect::~Kinect(void)
{
  // Kinect�I������
  if (kinect != 0) {
    kinect->NuiShutdown();  // Kinect�C���X�^���X�̓����~
    kinect->Release();      // Kinect�C���X�^���X�̉��
  }
}

/**
* @brief �������֐�
* @param[in] max_depth �[�x�̍ő�l
* @return true �����Cfalse ���s
*/
bool Kinect::initialize(float max_depth)
{
  this->max_depth = max_depth;

  // �ڑ�Kinect���̊m�F
  int num = 0;
  NuiGetSensorCount(&num);
  if (num == 0) {
    cerr << "error - cannot find Kinect" << endl;
    return false;
  }

  // Kinect�C���X�^���X�̐���
  NuiCreateSensorByIndex(0, &kinect);

  // Kinect�̏������i�[�x�J�������p�̐ݒ�j
  result = kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | 
    NUI_INITIALIZE_FLAG_USES_DEPTH);
  if (result != S_OK) {
    cerr <<"error - cannot initialize kinect" << endl;
    return false;
  }

  // �J���[�J�����̏������i�𑜓x��640x480�ɐݒ�j
  result = kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,
    NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &h_image);
  if (result != S_OK) {
    cerr <<"error - cannot open image stream of depth sensor" << endl;
    return false;
  }

  // �[�x�J�����̏������i�𑜓x��640x480�ɐݒ�j
  result = kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH,
    NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE,
    2, 0, &h_depth);
  if (result != S_OK) {
    cerr <<"error - cannot open image stream of depth sensor" << endl;
    return false;
  }

  // Near���[�h�̐ݒ�
  result = kinect->NuiImageStreamSetImageFrameFlags(h_depth,
    NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
  if (result != S_OK) {
    cerr <<"error - cannot set near mode" << endl;
    return false;
  }

  // 640x480�𑜓x�̉摜�T�C�Y���擾
  NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);

  return true;
}

/**
*
*/
void Kinect::colorAndDepthImage()
{
  // �X�g���[���C�x���g�̐���
  streamEvent = CreateEvent(0, TRUE, FALSE, 0);
  kinect->NuiSetFrameEndEvent(streamEvent, 0);

  // �ϐ��̏�����
  Mat color(height, width, CV_8UC3, Scalar(0));
  Mat depth(height, width, CV_8UC1, Scalar(0));
  Mat temp;

  const char *cname = "Color";
  const char *dname = "Depth";
  const char *rname = "Region";
  realpointsmm.resize(width * height);

  // �}�E�X�R�[���o�b�N�֐��̐ݒ�
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

    // �X�V�̐ݒ�
    WaitForSingleObject(streamEvent, INFINITE);
    ResetEvent(streamEvent);

    // �[�x�摜�̎擾
    NUI_IMAGE_FRAME depth_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_depth, INFINITE, &depth_frame);

    // �[�x�f�[�^�̎擾
    NUI_LOCKED_RECT depth_data = {0};
    depth_frame.pFrameTexture->LockRect( 0, &depth_data, 0, 0 );

    USHORT *depth_short = (USHORT*)depth_data.pBits;

    // ���Ԃ̋����̕��ς��v�Z
    float dist_ave = 0;
    int cnt = 0;

    for (int y = 0; y < depth.rows; y++) {
      for (int x = 0; x < depth.cols; x++, depth_short++) {
        // �[�x���̎擾
        Vector4 realpoints = NuiTransformDepthImageToSkeleton(x, y, 
          *depth_short, NUI_IMAGE_RESOLUTION_640x480);

        LONG depthX = x;
        LONG depthY = y;
        LONG colorX = depthX;
        LONG colorY = depthY;

        // �[�x���W���J���[�摜�̍��W�֕␳����
        kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
          NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, 0,
          depthX , depthY, *depth_short, &colorX, &colorY);

        int index = (colorY * width) + colorX;

        // m����mm�֕ϊ�
        realpointsmm.at(index).x = realpoints.x * 1000; 
        realpointsmm.at(index).y = realpoints.y * 1000;
        realpointsmm.at(index).z = realpoints.z * 1000;

        // 8bit�ɂ��Đ[�x�摜�Ɋi�[
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

    // �J���[�摜�̎擾
    NUI_IMAGE_FRAME image_frame = {0};
    kinect->NuiImageStreamGetNextFrame(h_image, INFINITE, &image_frame);

    NUI_LOCKED_RECT color_data;
    image_frame.pFrameTexture->LockRect(0, &color_data, 0, 0);

    // �t���[���f�[�^�̉��
    kinect->NuiImageStreamReleaseFrame(h_image, &image_frame);

    temp = Mat(height, width, CV_8UC4, color_data.pBits);
    cvtColor(temp, color, CV_BGRA2GRAY/*BGR*/);

    // ��l��
    cv::threshold(color, color, 100, 255, CV_THRESH_BINARY);
    cvtColor(color, color, CV_GRAY2BGR);

    // �d�S���v�Z���邽�ߗ̈敪��
    GraphBased gb(300, 10, 0, 0, 0);
    Mat flow = Mat::zeros(height, width, CV_32FC3);
    SubRegion region = gb.segmentImage(color, depth, flow);

    // �d�S�ʒu�̐���
    Mat output;
    int region_num = preprocess(label, region);
    output = drawSubRegion(label);
    calcCentroid(label, region_num);

    // �t���[���f�[�^�̉�� (release frame data)
    kinect->NuiImageStreamReleaseFrame(h_depth, &depth_frame);

    // �摜�o��
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
* @brief �}�E�X�R�[���o�b�N�֐�
*/
void Kinect::onMouse(int event, int x, int y, int flags, void *param)
{
  // ���N���b�N�C�x���g�������������ǂ���
  if (event == CV_EVENT_LBUTTONDOWN) {
    reinterpret_cast<Kinect *>(param)->read3DCoordinates(x, y);
  }

  // �E�N���b�N�C�x���g�������������ǂ���
  if (event == CV_EVENT_RBUTTONDOWN) {
    reinterpret_cast<Kinect *>(param)->readCentroid3DCoordinates(x, y);
  }
}

/**
* @brief �R�������W�̓ǂݎ��
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

  // �O�̌��ʂƂ̍����v�Z
  printf("diff : %f, %f, %f\n", abs(realpointsmm.at(index).x - prevpoint.x),
    abs(realpointsmm.at(index).y - prevpoint.y), abs(realpointsmm.at(index).z - prevpoint.z));

  // �O�̃N���b�N�̌��ʂƂ��Ċi�[
  prevpoint.x = realpointsmm.at(index).x;
  prevpoint.y = realpointsmm.at(index).y;
  prevpoint.z = realpointsmm.at(index).z;
}

/**
 * @brief �O�����i���x���̎擾�ƕ����̈�̋��E�𒊏o�j
 * @param[out] label   �����̈�̃��x��
 * @param[in] region  �����̈�
 * @return  �����̈�̐�
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

      // �����̈�̃��x�����擾
      label.at<int>(Point(x, y)) = tmp[comp];
    }
  }

  return region_num;
}

/**
* @brief �������ʂ̐F�t��
* @param[in] label  ���x��
* @return  �o�͌���
*/
Mat Kinect::drawSubRegion(Mat &label)
{
  int width = label.cols;
  int height = label.rows;

  // �̈敪���̌��ʂ�F�t��
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
 * @brief �e�̈�̏d�S���v�Z
 */
void Kinect::calcCentroid(const Mat &label, const int region_num)
{
  // �d�S���i�[����z��̃��������m��
  centroids.clear();
  centroids.resize(region_num);
  vector<int> size(region_num);

  // �d�S�̌v�Z
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