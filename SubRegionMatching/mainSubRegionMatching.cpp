/*****************************************************************************/
/*! @addtogroup 
 *  @file   mainSubRegionMatching.cpp
 *  @brief  �̈�n��̌�����s���v���O�����Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include <time.h> // �������Ԍv���p
#include "SubRegionMatching.h"
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

/* �v���g�^�C�v�錾 */
//void drawTemporalRegion(TemporalSubRegion &region, vector<Mat> &labels,
//  int start);
//void drawStreamingMatching(TemporalSubRegion &region, vector<Mat> &labels,
//  int s, vector<Vec3b> &rgb, int start, vector<int> &r_f, vector<int> &l,
//  vector<Features> &f_vec, float norm);
//void drawSubRegionSequence(TemporalSubRegion &region, vector<Mat> labels, 
//  int s, vector<Vec3b> &rgb, const int start, vector<int> &r_f, vector<int> &l,
//  vector<Features> &f_vec, float norm, vector<FeatureSequence> sequences, int min_frame);
//
//// �o�[�W�����Q
//void drawRegionSequence(TemporalSubRegion &region, vector<Mat> labels, 
//  int s, vector<Vec3b> &rgb, const int start, vector<int> &r_f, vector<int> &l,
//  vector<Features> &f_vec, float norm, vector<RegionSequence> region_seq);

/**
 * @brief ���C���֐�
 * @param[in] argc  �����̐� < 14
 * @param[in] argv  ����
 *  (1 : �ړ����镔���̈��臒l  2 : �����̈�̑Ή��t��臒l  
 *   3 : �����̃t���[���Ԋu�i�S�t���[���ł̏����𐄏��j
 *   4 : �擪�t���[���ԍ�  5 : �����t���[���ԍ�  6 : �t�@�C���̏o�� 
 *   7 : �����̈�̏d�S�E�傫���̏d�݌W��  8 : �摜�E�^���E�[�x���̏d�݌W��
 *   9 : �摜���̏d�݌W��  10 : �[�x���̏d�݌W��  11 : �^�����̏d�݌W��
 *   12 : �ŋߓ_�΂̐�  13 : �̈�n��̍ŏ��t���[����  14 : �̈�n��̍ŏ��̈ړ���
 */
int main(int argc, char *argv[])
{
  // �����̊m�F
  if (argc < 5) {
    cerr << "usage : matching norm weight interval start end file_out " <<
      "w_region w_rgbd w_I w_D w_F N min_frame min_move width height" << endl;
    return -1;
  }

  // �ϐ��錾
  float norm = atof(argv[1]);//1;       // �ړ��ʂ�臒l
  float w_threshold = atof(argv[2]); //60;   // �d�݂�臒l
  int interval = atoi(argv[3]);//100;   // �t���[���Ԋu
  int start = atoi(argv[4]);//0;        // �J�n�ԍ�
  int end = atoi(argv[5]);//9;          // �I���ԍ�

  bool file_out = false;
  float w_region = 1.0;
  float w_rgbd = 1.0;
  float w_I = 1.0;
  float w_D = 3.0;
  float w_F = 0.5;
  int N = 1.0;
  int min_frame = 0;
  float min_move = 0;
  int width = 640;
  int height = 480;

  if (argc >= 13) {
    if (atoi(argv[6]) == 1) {
      file_out = true;
      N = atoi(argv[12]);
      cout << "nearest point file output" << endl;
    }
  }

  if (argc >= 14) {
    min_frame = atoi(argv[13]);
    cout << "min frame num " << min_frame << endl;
  }
  if (argc >= 15) {
    min_move = atof(argv[14]);
    cout << "min move " << min_move << endl;
  }

  if (argc >= 10) {
    w_region = atof(argv[7]);
    w_rgbd = atof(argv[8]);
    w_I = atof(argv[9]);
    w_D = atof(argv[10]);
    w_F = atof(argv[11]);
  }
  else {
    cout << "set default weighting factor" << endl;
  }

  if (argc == 17) {
    width = atoi(argv[15]);
    height = atoi(argv[16]);
  }
  else {
    cout << "set default image size 640x480" << endl;
  }

  _mkdir("./out_matching");

  /////////////////////////////////////////////////////////////////////////////
  // �X�g���[�~���O����
  SubRegionMatching matching(start, end, norm, w_threshold, 
    w_region, w_rgbd, w_I, w_D, w_F);

  clock_t s_time, e_time; // �������Ԍv���p
  FILE *fp;
  fp = fopen("./matching-time.dat", "w");

  Features result;
  int s = start, e;

  vector<Vec3b> rgb;
  rgb.resize(height * width * 30);
  for (int i = 0; i < height * width * 30; i++) {
    rgb[i] = Vec3b(rand() % 256, rand() % 256, rand() % 256);
  }
  vector<int> r_f, l;

  for(;;) {
    if (s == end) {
      break;
    }

    e = min(s + interval, end);

    // �t�@�C���̓ǂݍ���
    vector<Features> f_vec = readMultipleFeaturesDat(s, e);
    vector<Mat> l_vec;
    int tmps;
    if (result.empty()) {
      tmps = s;
      l_vec = readMultipleLabelDat(tmps, e, width, height);
    }
    else {
      tmps = s - 1;
      l_vec = readMultipleLabelDat(tmps, e, width, height);
    }

    cout << "matching frame : " << tmps << "--" << e - 1 << endl;

    s_time = clock();

    /*TemporalSubRegion fregion, bregion;
    matching.bidirectionalSubRegionMatching(fregion, bregion, f_vec, l_vec);*/

    // �����̈�̑Ή��t���i�̈�n��̌���j
    RegionSequences region_seq = matching.streamingSubRegionMatching(f_vec,
      l_vec, s, e, result);
    cout << "sequence num (before remove) : " << region_seq.size() << endl;

    // �Z���̈�n��C�ړ��̏��Ȃ��̈�n�������
    region_seq = matching.removeSequences(region_seq, min_frame, min_move);
    
    // �Ή��t�����ʂ̕`��
    //drawStreamingMatching(region, l_vec, tmps, rgb, start, r_f, l,
    //  f_vec, norm);

    // �t�@�C���̏�������
    //vector<FeatureSequence> sequences = 
    //  writeSubRegionSequencesDat(region, f_vec, l_vec, N, tmps, e - 1,
    //  min_frame, min_move, file_out);
    //cout << "sequence num : " << sequences.size() << endl << endl;
    //drawSubRegionSequence(region, l_vec, tmps, rgb, start, r_f, l,
    //  f_vec, norm, sequences, min_frame);

    //region_seq = matching.regionToSequence2(fregion, bregion,
    //  f_vec, min_frame, min_move);
      //writeRegionSequencesDat(region, f_vec, l_vec, N, tmps, e - 1,
      //min_frame, min_move, file_out);
    cout << "sequence num (before segment): " << region_seq.size() << endl;

    ///////////////////////////////////////////////////////////////////////////
    // �̈�n��̕��������i�o�O�����肻���ȓ�����s���Ă��邽�ߗv�C���j
    float threshold = 20;   // �̈�n����̑���x��臒l
    int iteration = 0;     // �̈�n�񕪊������̔�����
    for (int i = 0; i < iteration; i++) {
      RegionSequences tmp_seq;
      for (int i = 0; i < region_seq.size(); i++) {
        RegionSequences tmp = matching.sequenceSegmentation(region_seq.at(i), 
          l_vec, N, threshold);
        tmp_seq.insert(tmp_seq.end(), tmp.begin(), tmp.end());
      }
      region_seq = tmp_seq;

      cout << "sequence num (before remove): " << region_seq.size() << endl;

      // �Z���̈�n��C�ړ��̏��Ȃ��̈�n�������
      region_seq = matching.removeSequences(region_seq, min_frame, min_move);
      cout << "sequence num : " << region_seq.size() << endl << endl;
    }
    ///////////////////////////////////////////////////////////////////////////

    // ���Ԃ̌v��
    e_time = clock();
    cout << "subregion matching : " << e_time - s_time << "[ms]" << endl;
    fprintf(fp, "subregion matching : %d[ms]\n", e_time - s_time);
    
    // �̈�n��̕`��
    for (int i = 0; i < l_vec.size(); i++) {
      Mat color = drawRegionSequence(getFrameFeatures(region_seq, i), l_vec.at(i), rgb, i);
      Mat move = drawMovingSubRegion(l_vec.at(i), f_vec.at(i), norm);

      // �ۑ��p
      char name[80];
      string buf;
      sprintf(name, "./out_matching/matching-0%004d.png", i + tmps);
      buf = name;
      imwrite(buf, color);

      sprintf(name, "./out_matching/moving-0%004d.png", i + tmps);
      buf = name;
      imwrite(buf, move);

      imshow("color", color);
      imshow("moving", move);
      waitKey(10);
    }

    destroyAllWindows();

    // �t�@�C���̏�������
    if (file_out) {
      writeRegionSequencesDat(region_seq, tmps, e - 1);
      writeNearestPointDat(region_seq, l_vec, N, tmps, e - 1);
    }

    s = e;
  }

  // �t�@�C���̃N���[�Y
  fclose(fp);

  /////////////////////////////////////////////////////////////////////////////


  //// �t�@�C���̓ǂݍ���
  //vector<Mat> l_vec = readMultipleLabelDat(start, end, width, height);
  //vector<Features> f_vec = readMultipleFeaturesDat(start, end);
  //
  //clock_t s_time, e_time; // �������Ԍv���p
  //FILE *fp;
  //fp = fopen("./matching-time.dat", "w");

  //s_time = clock();

  //// �����̈�̑Ή��t��
  //SubRegionMatching matching(start, end, norm, dist, weight);
  //TemporalSubRegion region = matching.subRegionMatching(f_vec);
  //
  //// ���Ԃ̌v��
  //e_time = clock();
  //cout << "subregion matching : " << e_time - s_time << "[ms]" << endl;
  //fprintf(fp, "subregion matching : %d[ms]\n", e_time - s_time);

  //// �Ή��t�����ʂ̕`��
  //drawTemporalRegion(region, l_vec, start);
  //
  //// �t�@�C���̏�������
  //int num = writeSubRegionSequencesDat(region, f_vec, l_vec, start);
  //cout << num << endl;
  //fprintf(fp, "subregion sequences num : %d\n", num);

  //// �t�@�C���̃N���[�Y
  //fclose(fp);

  return 0;
}

///**
// * @brief ���n��摜�̐F�t��
// */
//void drawTemporalRegion(TemporalSubRegion &region, vector<Mat> &labels, int start)
//{
//  char name[80];
//
//  int num = labels.size();
//  int width = labels.at(0).cols;
//  int height = labels.at(0).rows;
//
//  vector<Vec3b> rgb(width * height * num);
//  for (int i = 0; i < width * height * num; i++) {
//    rgb[i] = Vec3b(rand() % 256, rand() % 256, rand() % 256);
//  }
//
//  for (int z = 0; z < num; z++) {
//    Mat output = ~Mat::zeros(height, width, CV_8UC3);
//    for (int y = 0; y < height; y++) {
//      for (int x = 0; x < width; x++) {
//        int element = labels.at(z).at<int>(Point(x, y));
//        int f, p, rank, r_prev;
//        region.find(z, element, f, p, rank, r_prev);
//
//        if ((rank != 0 || r_prev != 0) && region.getNorm(f, p)) {
//          output.at<Vec3b>(Point(x, y)) = rgb[(f + 1) * p + 1];
//        }
//      }
//    }
//
//    // �ۑ��p
//    string buf;
//    sprintf(name, "./out_matching/matching-0%004d.png", z + start);
//    buf = name;
//    imwrite(buf, output);
// 
//    imshow("color", output);
//    waitKey(100);
//  }
//}
//
//void drawStreamingMatching(TemporalSubRegion &region, vector<Mat> &labels,
//  int s, vector<Vec3b> &rgb, int start, vector<int> &real_frame, vector<int> &l,
//  vector<Features> &f_vec, float norm)
//{
//  char name[80];
//
//  int num = labels.size();
//  int width = labels.at(0).cols;
//  int height = labels.at(0).rows;
//
//  for (int z = 0; z < num; z++) {
//    Mat output = ~Mat::zeros(height, width, CV_8UC3);
//    for (int y = 0; y < height; y++) {
//      for (int x = 0; x < width; x++) {
//        int element = labels.at(z).at<int>(y, x);
//        int f, p, rank, r_prev, r_f;
//        r_f = region.find(z, element, f, p, rank, r_prev);
//
//        if (region.getNorm(f, p)) {
////        if ((rank != 0 || r_prev != 0) && region.getNorm(f, p)) {
//          if (real_frame.empty() && l.empty()) {
//            output.at<Vec3b>(y, x) = rgb[(r_f - start + 1) * p + 1];
//          }
//          else {
//            if (r_f == s) {
//              output.at<Vec3b>(y, x) = rgb[(real_frame.at(p) - start + 1) * l.at(p) + 1];
//            }
//            else {
//              output.at<Vec3b>(y, x) = rgb[(r_f - start + 1) * p + 1];
//            }
//          }
//        }
//      }
//    }
//
//    ///////////////////////////////////////////////////////////////////////////
//    // �ړ����镔���̈�̕`��
//    Mat move = ~Mat::zeros(height, width, CV_8UC3);
//
//    // �t���[�̕`��
//    for (int y = 0; y < height; y++) {
//      for (int x = 0; x < width; x++) {
//        int l = labels.at(z).at<int>(y, x);
//        if (f_vec.at(z).at(l).norm3d > norm) {
//          move.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
//        }
//      }
//    }
//
//    // �����̈�̋��E��`��
//    for (int y = 0; y < height - 1; y++) {
//      for (int x = 0; x < width - 1; x++) {
//        int comp1 = labels.at(z).at<int>(y, x);
//
//        for (int yy = y; yy < y + 2; yy++) {
//          for (int xx = x; xx < x + 2; xx++) {
//            int comp2 = labels.at(z).at<int>(yy, xx);
//            if (comp1 != comp2) {
//              move.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
//            }
//          }
//        }
//      }
//    }
//    ///////////////////////////////////////////////////////////////////////////
//
//    // �ۑ��p
//    string buf;
//    sprintf(name, "./out_matching/matching-0%004d.png", z + s);
//    buf = name;
//    imwrite(buf, output);
// 
//    imshow("color", output);
//    imshow("moving", move);
//    waitKey(100);
//  }
//
//  // �Ō�̃t���[���̑Ή��t�����ʂ��擾
//  vector<int> tmp_frame = real_frame;
//  vector<int> tmp_label = l;
//  region.getLast(real_frame, l);
//
//  // �O�̌��ʂ̃t���[���ԍ��ƃ��x���𔽉f
//  if (tmp_frame.empty()) return;
//
//  int size = real_frame.size();
//  for (int i = 0; i < size; i++) {
//    if (real_frame.at(i) == s) {
//      int index = l.at(i);
//      real_frame.at(i) = tmp_frame.at(index);
//      l.at(i) = tmp_label.at(index);
//    }
//  }
//}
//
///**
// * @brief sequence���g���čŏ��̕����̈�n�񐔂𒴂���̈�݂̂�Ή��t��
// */
//void drawSubRegionSequence(TemporalSubRegion &region, vector<Mat> labels,
//  int s, vector<Vec3b> &rgb, const int start, vector<int> &real_frame, vector<int> &l,
//  vector<Features> &f_vec, float norm, vector<FeatureSequence> sequences,
//  int min_frame)
//{
//  char name[80];
//  
//  int num = labels.size();
//  int width = labels.at(0).cols;
//  int height = labels.at(0).rows;
//
//  for (int z = 0; z < num; z++) {
//    ///////////////////////////////////////////////////////////////////////////
//    // �}�b�`���O���ʂ̃��x��
//    sprintf(name, "./subregion/mlabel-0%004d.dat", z + s);
//    FILE *fpm = fopen(name, "w");
//    // �}�b�`���O���ʂ̃t���[��
//    sprintf(name, "./subregion/flabel-0%004d.dat", z + s);
//    FILE *fpf = fopen(name, "w");
//    ///////////////////////////////////////////////////////////////////////////
//
//
//    Mat output = ~Mat::zeros(height, width, CV_8UC3);
//    for (int j = 0; j < height * width; j++) {
//      if (j != 0 && j % width == 0) {
//        fprintf(fpm, "\n");
//        fprintf(fpf, "\n");
//      }
//
//      int element = labels.at(z).at<int>(j);
//      int f, p, rank, r_prev, r_f;
//
//      bool file_flag = false;
//
//      r_f = region.find(z, element, f, p, rank, r_prev);
//      if (region.getNorm(f, p)) {
//        if (real_frame.empty() && l.empty()) {
//          for (int k = 0; k < sequences.size(); k++) {
//            if (sequences.at(k).getFrame() == f && sequences.at(k).getLabel() == p
//              && sequences.at(k).getLength() > min_frame) 
//            {
//              output.at<Vec3b>(j) = rgb[(f + 1) * p + 1];
//
//              fprintf(fpm, "%d ", sequences.at(k).getLabel());
//              fprintf(fpf, "%d ", sequences.at(k).getFrame());
//
//              file_flag = true;
//            }
//          }
//        }
//        else {
//          // �X�g���[�~���O�����Ōn��̌���Ɏ኱�o�O�����肻��
//          if (r_f == s) {
//            for (int k = 0; k < sequences.size(); k++) {
//              if (sequences.at(k).getFrame() == real_frame.at(p) - start && sequences.at(k).getLabel() == p
//                && sequences.at(k).getLength() > min_frame)
//              {
//                output.at<Vec3b>(j) = rgb[(real_frame.at(p) - start + 1) * l.at(p) + 1];
//
//                fprintf(fpm, "%d ", sequences.at(k).getLabel());
//                fprintf(fpf, "%d ", sequences.at(k).getFrame());
//
//                file_flag = true;
//              }
//            }
//
//            if (!file_flag) {
//              for (int k = 0; k < sequences.size(); k++) {
//                if (sequences.at(k).getFrame() == f && sequences.at(k).getLabel() == p
//                  && sequences.at(k).getLength() > min_frame)
//                {
//                  output.at<Vec3b>(j) = rgb[(real_frame.at(p) - start + 1) * l.at(p) + 1];
//                  
//                  fprintf(fpm, "%d ", sequences.at(k).getLabel());
//                  fprintf(fpf, "%d ", sequences.at(k).getFrame());
//
//                  file_flag = true;
//                }
//              }
//            }
//          }
//          else {
//            for (int k = 0; k < sequences.size(); k++) {
//              if (sequences.at(k).getFrame() == f && sequences.at(k).getLabel() == p
//                && sequences.at(k).getLength() > min_frame)
//              {
//                output.at<Vec3b>(j) = rgb[(f + 1) * p + 1];
//
//                fprintf(fpm, "%d ", sequences.at(k).getLabel());
//                fprintf(fpf, "%d ", sequences.at(k).getFrame());
//
//                file_flag = true;
//              }
//            }
//          }
//        }
//      }
//
//      if (!file_flag) {
//        fprintf(fpm, "-1 ");
//        fprintf(fpf, "-1 ");
//      }
//    }
//
//    ///////////////////////////////////////////////////////////////////////////
//    // �ړ����镔���̈�̕`��
//    Mat move = ~Mat::zeros(height, width, CV_8UC3);
//
//    // �t���[�̕`��
//    for (int y = 0; y < height; y++) {
//      for (int x = 0; x < width; x++) {
//        int l = labels.at(z).at<int>(y, x);
//        if (f_vec.at(z).at(l).norm3d > norm) {
//          move.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
//        }
//      }
//    }
//
//    // �����̈�̋��E��`��
//    for (int y = 0; y < height - 1; y++) {
//      for (int x = 0; x < width - 1; x++) {
//        int comp1 = labels.at(z).at<int>(y, x);
//
//        for (int yy = y; yy < y + 2; yy++) {
//          for (int xx = x; xx < x + 2; xx++) {
//            int comp2 = labels.at(z).at<int>(yy, xx);
//            if (comp1 != comp2) {
//              move.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
//            }
//          }
//        }
//      }
//    }
//    ///////////////////////////////////////////////////////////////////////////
//
//    // �ۑ��p
//    string buf;
//    sprintf(name, "./out_matching/matching-0%004d.png", z + s);
//    buf = name;
//    imwrite(buf, output);
//
//    sprintf(name, "./out_matching/moving-0%004d.png", z + s);
//    buf = name;
//    imwrite(buf, move);
// 
//    imshow("color", output);
//    imshow("moving", move);
//    waitKey(100);
//
//    fclose(fpm);
//    fclose(fpf);
//  }
//
//  // �Ō�̃t���[���̑Ή��t�����ʂ��擾
//  vector<int> tmp_frame = real_frame;
//  vector<int> tmp_label = l;
//  region.getLast(real_frame, l);
//
//  // �O�̌��ʂ̃t���[���ԍ��ƃ��x���𔽉f
//  if (tmp_frame.empty()) return;
//
//  int size = real_frame.size();
//  for (int i = 0; i < size; i++) {
//    if (real_frame.at(i) == s) {
//      int index = l.at(i);
//      real_frame.at(i) = tmp_frame.at(index);
//      l.at(i) = tmp_label.at(index);
//    }
//  }
//}
//
//
///**
// * @brief region_seq���g���čŏ��̕����̈�n�񐔂𒴂���̈�݂̂�Ή��t��
// */
//void drawRegionSequence(TemporalSubRegion &region, vector<Mat> labels,
//  int s, vector<Vec3b> &rgb, const int start, vector<int> &real_frame, vector<int> &l,
//  vector<Features> &f_vec, float norm, vector<RegionSequence> region_seq)
//{
//  char name[80];
//  
//  int num = labels.size();
//  int width = labels.at(0).cols;
//  int height = labels.at(0).rows;
//
//  for (int z = 0; z < num; z++) {
//    ///////////////////////////////////////////////////////////////////////////
//    // �}�b�`���O���ʂ̃��x��
//    sprintf(name, "./subregion/mlabel-0%004d.dat", z + s);
//    FILE *fpm = fopen(name, "w");
//    // �}�b�`���O���ʂ̃t���[��
//    sprintf(name, "./subregion/flabel-0%004d.dat", z + s);
//    FILE *fpf = fopen(name, "w");
//    ///////////////////////////////////////////////////////////////////////////
//
//
//    Mat output = ~Mat::zeros(height, width, CV_8UC3);
//    for (int j = 0; j < height * width; j++) {
//      if (j != 0 && j % width == 0) {
//        fprintf(fpm, "\n");
//        fprintf(fpf, "\n");
//      }
//
//      int element = labels.at(z).at<int>(j);
//      int f, p, rank, r_prev, r_f;
//
//      bool file_flag = false;
//
//      r_f = region.find(z, element, f, p, rank, r_prev);
//      if (region.getNorm(f, p)) {
//        if (real_frame.empty() && l.empty()) {
//          for (int k = 0; k < region_seq.size(); k++) {
//            if (region_seq.at(k).getLabel() == TemporalVertex(f, p)) {
//              output.at<Vec3b>(j) = rgb[(f + 1) * p + 1];
//
//              fprintf(fpm, "%d ", region_seq.at(k).getLabel().label);
//              fprintf(fpf, "%d ", region_seq.at(k).getLabel().frame);
//
//              file_flag = true;
//            }
//          }
//        }
//        else {
//          // �X�g���[�~���O�����Ōn��̌���Ɏ኱�o�O�����肻��
//          if (r_f == s) {
//            for (int k = 0; k < region_seq.size(); k++) {
//              if (region_seq.at(k).getLabel() == 
//                TemporalVertex(real_frame.at(p) - start, p))
//              {
//                output.at<Vec3b>(j) = rgb[(real_frame.at(p) - start + 1) * l.at(p) + 1];
//
//                fprintf(fpm, "%d ", region_seq.at(k).getLabel().label);
//                fprintf(fpf, "%d ", region_seq.at(k).getLabel().frame);
//
//                file_flag = true;
//              }
//            }
//
//            if (!file_flag) {
//              for (int k = 0; k < region_seq.size(); k++) {
//                if (region_seq.at(k).getLabel() == TemporalVertex(f, p))
//                {
//                  output.at<Vec3b>(j) = rgb[(real_frame.at(p) - start + 1) * l.at(p) + 1];
//                  
//                  fprintf(fpm, "%d ", region_seq.at(k).getLabel().label);
//                  fprintf(fpf, "%d ", region_seq.at(k).getLabel().frame);
//
//                  file_flag = true;
//                }
//              }
//            }
//          }
//          else {
//            for (int k = 0; k < region_seq.size(); k++) {
//              if (region_seq.at(k).getLabel() == TemporalVertex(f, p))
//              {
//                output.at<Vec3b>(j) = rgb[(f + 1) * p + 1];
//
//                fprintf(fpm, "%d ", region_seq.at(k).getLabel().label);
//                fprintf(fpf, "%d ", region_seq.at(k).getLabel().frame);
//
//                file_flag = true;
//              }
//            }
//          }
//        }
//      }
//
//      if (!file_flag) {
//        fprintf(fpm, "-1 ");
//        fprintf(fpf, "-1 ");
//      }
//    }
//
//    ///////////////////////////////////////////////////////////////////////////
//    // �ړ����镔���̈�̕`��
//    Mat move = ~Mat::zeros(height, width, CV_8UC3);
//
//    // �t���[�̕`��
//    for (int y = 0; y < height; y++) {
//      for (int x = 0; x < width; x++) {
//        int l = labels.at(z).at<int>(y, x);
//        if (f_vec.at(z).at(l).norm3d > norm) {
//          move.at<Vec3b>(y, x) = Vec3b(0, 0, 255);
//        }
//      }
//    }
//
//    // �����̈�̋��E��`��
//    for (int y = 0; y < height - 1; y++) {
//      for (int x = 0; x < width - 1; x++) {
//        int comp1 = labels.at(z).at<int>(y, x);
//
//        for (int yy = y; yy < y + 2; yy++) {
//          for (int xx = x; xx < x + 2; xx++) {
//            int comp2 = labels.at(z).at<int>(yy, xx);
//            if (comp1 != comp2) {
//              move.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
//            }
//          }
//        }
//      }
//    }
//    ///////////////////////////////////////////////////////////////////////////
//
//    // �ۑ��p
//    string buf;
//    sprintf(name, "./out_matching/matching-0%004d.png", z + s);
//    buf = name;
//    imwrite(buf, output);
//
//    sprintf(name, "./out_matching/moving-0%004d.png", z + s);
//    buf = name;
//    imwrite(buf, move);
// 
//    imshow("color", output);
//    imshow("moving", move);
//    waitKey(100);
//
//    fclose(fpm);
//    fclose(fpf);
//  }
//
//  // �Ō�̃t���[���̑Ή��t�����ʂ��擾
//  vector<int> tmp_frame = real_frame;
//  vector<int> tmp_label = l;
//  region.getLast(real_frame, l);
//
//  // �O�̌��ʂ̃t���[���ԍ��ƃ��x���𔽉f
//  if (tmp_frame.empty()) return;
//
//  int size = real_frame.size();
//  for (int i = 0; i < size; i++) {
//    if (real_frame.at(i) == s) {
//      int index = l.at(i);
//      real_frame.at(i) = tmp_frame.at(index);
//      l.at(i) = tmp_label.at(index);
//    }
//  }
//}