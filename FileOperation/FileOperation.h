/*****************************************************************************/
/*! @addtogroup 
 *  @file   FileOperation.h
 *  @brief  �t�@�C������Ɋւ���t�@�C���i�w�b�_�t�@�C���j
 *  @date   
 *  @author ksugawara
******************************************************************************/
#ifndef FILEOPERATION_H
#define FILEOPERATION_H

#include <time.h> // �������Ԍv���p
#include <direct.h> // �t�H���_�쐬�p
#include <fstream>
#include <istream>
#include "../Miscellaneous/Misc.h"
#include "WriteNPThread.h"

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// �o�[�W�������̎擾
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// lib�t�@�C�����̍Ō�̕�����Release��Debug�ŕ�����
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#pragma comment(lib, "../x64/Debug/Miscellaneousd.lib")
#else
#define CV_EXT_STR ".lib"
#pragma comment(lib, "../x64/Release/Miscellaneous.lib")
#endif

#endif

/* �萔 */
#define MAX_LABEL   8000  
#define MAX_FEATURE 300   

/*! @namespace FileOperation
 *  @brief  �t�@�C������p�̖��O���
 */
namespace FileOperation
{
  // �������ݏ���
  void writeLabelDat(const Mat &label, const int num);
  void writeFeaturesDat(const Features &features, const int num);
  void writeSceneFlowDat(const Mat &flow, const int num);

  /*vector<FeatureSequence> writeSubRegionSequencesDat(TemporalSubRegion &region, vector<Features> &f_vec,
    vector<Mat> &labels, const int N, const int start, const int end, 
    const int min_frame, const float min_move, bool flag);*/

  void writeRegionSequencesDat(RegionSequences &region_seq, const int start, const int end);
  void writeNearestPointDat(RegionSequences &region_seq, vector<Mat> &labels, 
    const int N, const int start, const int end);
  
  // �ǂݍ��ݏ���
  Mat readLabelDat(const int num, const int width, const int height);
  vector<Mat> readMultipleLabelDat(const int start, const int end, const int width,
    const int height);
  Features readFeaturesDat(const int num);
  vector<Features> readMultipleFeaturesDat(const int start, const int end);
  Mat readSceneFlowDat(const int num, const int width, const int height);
  vector<Mat> readMultipleSceneFlowDat(const int start, const int end, 
    const int width, const int height);

  //vector<FeatureSequence> readMultipleSequencesDat(void);
  //vector<FeatureSequence> readMultipleSequencesDat(const int start, const int end);
  RegionSequences readRegionSequencesDat(const int start, const int end);
  
  //Mat readDistanceDat(const int num, const int subregion_num);
  //vector<Mat> readMultipleDistanceDat(const int start, const int end, 
  //  const int subregion_num);

  //void readDistancePointDat(const int num, const int x, const int y,
  //  vector<Point> &pt1, vector<Point> &pt2);
  void readDistancePointDat(const int num, const int x, const int y,
    vector<Point> &pt1, vector<Point> &pt2, const int start, const int end);
}

#endif