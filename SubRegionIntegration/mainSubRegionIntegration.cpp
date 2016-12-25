/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainSubRegionIntegration.cpp
 *  @brief  �̈�n��̓������s���v���O�����Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "SubRegionIntegration.h"

#include "../DrawRegion/DrawRegion.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/DrawRegiond.lib")
#else
#pragma comment(lib, "../x64/Release/DrawRegion.lib")
#endif

using namespace DrawRegion;

///* �v���g�^�C�v�錾 */
//void drawIntegrationRegion(SubRegion region, RegionSequences &sequences, 
//  int start, int frame_num, int width, int height);

/**
 * @brief ���C���֐�
 * @param[in] argc  �����̐� < 8
 * @param[in] argv  ����
 *  (1:�̈�n��̓�����臒l 2:3�����ʒu�̏d�݌W�� 3:�V�[���t���[�̏d�݌W��
 *   4:�擪�t���[���ԍ� 5:�����t���[���ԍ� 6:�ŋߓ_�΂𗘗p 
 *   7:�̈�n��Ԃ̐ڐG�񐔂�臒l(0����)) 8:���� 9:�c��
 */
int main(int argc, char *argv[])
{
  // �����̊m�F
  if (argc < 8) {
    cerr << "usage : integration dist w_locate w_flow start end near_flag min_contact width height" << endl;
    return -1;
  }

  // �ϐ��錾
  char name[80];
  float dist = atof(argv[1]);       // ����x��臒l
  float w_locate = atof(argv[2]);   // �ʒu�̏d�݌W��
  float w_flow = atof(argv[3]);     // �����i�V�[���t���[�j�̏d�݌W��
  int start = atoi(argv[4]);        // �擪�t���[���ԍ�
  int end = atoi(argv[5]);          // �����t���[���ԍ�
  
  // �ŋߓ_�΂𗘗p
  bool near_flag = true;
  if (atoi(argv[6]) == 0) {
    near_flag = false;
  }

  int min_contact = atoi(argv[7]);

  int width = 640;
  int height = 480;

  if (argc == 10) {
    width = atoi(argv[8]);
    height = atoi(argv[9]);
  }
  else {
    cout << "set default image size 640x480" << endl;
  }

  clock_t s_time, e_time; // �������Ԍv��
  vector<Vec3b> rgb;
  rgb.resize(height * width * 30);
  for (int i = 0; i < height * width * 30; i++) {
    rgb[i] = Vec3b(rand() % 256, rand() % 256, rand() % 256);
  }

  // �t�@�C���̓ǂݍ���
  RegionSequences region_seq = readRegionSequencesDat(start, end);
  vector<Mat> flows = readMultipleSceneFlowDat(start, end, width, height);  // ���Ȃ�x��
  vector<Mat> labels = readMultipleLabelDat(start, end, width, height);

  cout << "start sequence integration process" << endl;
  s_time = clock();

  cout << "prev " << region_seq.size() << endl;

  // �̈�n��̓���
  SubRegionIntegration integration(dist, w_locate, w_flow, 
    width, height, start, end, near_flag, min_contact);
  region_seq = integration.integrateSequence(region_seq, flows);

  // �����ɐڐG�E�������s���Ȃ��̈�n����������鏈����ǉ�

  // �e�t���[���ł̕����̈�̒��o���Ԃ��v��
  e_time = clock();
  cout << "next " << region_seq.size() << endl;
  cout << "sequence integration time : " << e_time - s_time << "[ms]" << endl;

  // �������ʂ̕`��
  /*drawIntegrationRegion(region, region_seq, start, min_sequence, flows.size(),
    width, height);*/
  _mkdir("out_integration");
  for (int t = 0; t < labels.size(); t++) {
    Mat output = drawRegionSequence(getFrameFeatures(region_seq, t), labels.at(t), rgb, t);
    string buf;
    sprintf(name, "./out_integration/integration-0%004d.png", t + start);
    buf = name;
    imwrite(buf, output);
    imshow("output", output);
    waitKey(10);
  }

  cv::destroyAllWindows();

  return 0;
}

///**
// * @brief �̈�n��̓������ʂ�`��
// */
//void drawIntegrationRegion(SubRegion region, RegionSequences &sequences, 
//  int start, int frame_num, int width, int height)
//{
//
//  _mkdir("./out_integration/");
//
//  char name[80];
//
//  vector<Vec3b> rgb(width * height * frame_num);
//  for (int i = 0; i < width * height * frame_num; i++) {
//    rgb.at(i) = Vec3b(rand() % 256, rand() % 256, rand() % 256);
//  }
//
//  // ���x���̓ǂݍ���
//  FILE *fpm, *fpf;
//  char readline[MAX_LABEL] = {'\0'};
//
//  for (int i = 0; i < frame_num; i++) {
//    sprintf(name, "./subregion/mlabel-0%004d.dat", i + start);
//    fpm = fopen(name, "r");
//
//    sprintf(name, "./subregion/flabel-0%004d.dat", i + start);
//    fpf = fopen(name, "r");
//
//    Mat mlabel = Mat::zeros(height, width, CV_32SC1);
//    Mat flabel = Mat::zeros(height, width, CV_32SC1);
//
//    for (int y = 0; fgets(readline, MAX_LABEL, fpm) != NULL; y++) {
//      mlabel.at<int>(y, 0) = atoi(strtok(readline, " "));
//      for (int x = 1; x < width; x++) {
//        mlabel.at<int>(y, x) = atoi(strtok(NULL, " "));
//      }
//    }
//
//    for (int y = 0; fgets(readline, MAX_LABEL, fpf) != NULL; y++) {
//      flabel.at<int>(y, 0) = atoi(strtok(readline, " "));
//      for (int x = 1; x < width; x++) {
//        flabel.at<int>(y, x) = atoi(strtok(NULL, " "));
//      }
//    }
//
//    fclose(fpm);
//    fclose(fpf);
//
//    Mat output = ~Mat::zeros(height, width, CV_8UC3);
//
//    for (int j = 0; j < height * width; j++) {
//      int frame = flabel.at<int>(j);
//      int label = mlabel.at<int>(j);
//      
//      for (int k = 0; k < sequences.size(); k++) {
//        if (sequences.at(k).getLabel() == TemporalVertex(frame, label)) {
//          output.at<Vec3b>(j) = 
//            rgb.at((sequences.at(region.find(k)).getLabel().frame + 1) 
//            * sequences.at(region.find(k)).getLabel().label);
//        }
//      }
//    }
//
//    // �ۑ�
//    string buf;
//    sprintf(name, "./out_integration/integration-0%004d.png", i + start);
//    buf = name;
//    imwrite(buf, output);
//
//    imshow("integration", output);
//    waitKey(10);
//  }
//}