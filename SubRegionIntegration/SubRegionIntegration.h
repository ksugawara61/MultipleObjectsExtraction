/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionIntegration.h
 *  @brief  �̈�n��̓��������Ɋւ���t�@�C���i�w�b�_�t�@�C���j
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef SUBREGIONINTEGRATION_H
#define SUBREGIONINTEGRATION_H

#include "../GraphBased/GraphBased.h"
#include "../Miscellaneous/Misc.h"
#include "../FileOperation/FileOperation.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/GraphBasedd.lib")
#pragma comment(lib, "../x64/Debug/Miscellaneousd.lib")
#pragma comment(lib, "../x64/Debug/FileOperationd.lib")
#else
#pragma comment(lib, "../x64/Release/GraphBased.lib")
#pragma comment(lib, "../x64/Release/Miscellaneous.lib")
#pragma comment(lib, "../x64/Release/FileOperation.lib")
#endif

using namespace FileOperation;

/*! @class  SubRegionIntegration
 *  @brief  �̈�n��̓��������Ɋւ���N���X
 */
class SubRegionIntegration
{
public:
  SubRegionIntegration(const float threshold, const float w_locate, const float w_flow,
    const int width, const int height, const int start, const int end, bool near_flag,
    const int min_contact);
  ~SubRegionIntegration(void);
  RegionSequences integrateSequence(RegionSequences sequences, const vector<Mat> &flows);

private:
  float sequencesDistance(const vector<Mat> &flows, 
    const vector<RegionSequence> &s, const int x, const int y);
  float subregionsDistance(const Mat &flow, const vector<Point> &pt1, 
    const vector<Point> &pt2, const int num);

  // �����̈�̓����Ɋ�Â��������v�Z
  float subregionBasedDistance(const vector<Point> &pt1, 
    const vector<Point> &pt2, const Features f1, const Features f2, const int num);

  bool contactDetection(RegionSequence seq1, RegionSequence seq2, const vector<Mat> &flows);

  //// Cosine�ވȓx�ł̌v�Z
  //float sequenceCosineSimilarity(const vector<Mat> &flows, 
  //  const vector<FeatureSequence> &s, const int x, const int y);
  //float cosineSimilarity(const Mat &flow, const vector<Point> &pt1, 
  //  const vector<Point> &pt2, const int num);

  //// �̈�Ԃ̈ʒu�̗ވȓx
  //float sequenceLocation(const vector<FeatureSequence> &s, const int x, const int y);
  //float subregionLocation(const vector<Point> &pt1, const vector<Point> &pt2, const int num);

  //// �̈�Ԃ̃V�[���t���[�̗ވȓx
  //float sequenceSceneFlow(const vector<Mat> &flows, 
  //  const vector<FeatureSequence> &s, const int x, const int y);
  //float subregionSceneFlow(const Mat &flow, const vector<Point> &pt1, 
  //  const vector<Point> &pt2, const int num);

  //float averageDistance(const vector<Mat> &distmat, const int x, const int y);
  //float stdevDistance(const vector<Mat> &distmat, const int x, const int y);
  //float weightSequences(const FeatureSequence &f1, const FeatureSequence &f2);


  float threshold;    // �̈�n��Ԃ̑���x��臒l
  int min_contact;    // �̈�n��̍ŏ��ڐG��
  float w_locate; // �ʒu�̏d��
  float w_flow;   // �V�[���t���[�̏d��
  int width;
  int height;
  int start;    // �擪�̃t���[���ԍ�
  int end;      // �����̃t���[���ԍ�

  bool near_flag;  // �ŋߓ_�̑΂𗘗p���邩
};

#endif