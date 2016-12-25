/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionMatching.h
 *  @brief  �̈�n��̌��菈���Ɋւ���t�@�C���i�w�b�_�t�@�C���j
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef SUBREGIONMATCHING_H
#define SUBREGIONMATCHING_H

#include <time.h> // �������Ԍv���p
#include "../Miscellaneous/Misc.h"
#include "../FileOperation/FileOperation.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/Miscellaneousd.lib")
#pragma comment(lib, "../x64/Debug/FileOperationd.lib")
#else
#pragma comment(lib, "../x64/Release/Miscellaneous.lib")
#pragma comment(lib, "../x64/Release/FileOperation.lib")
#endif

using namespace FileOperation;

/*! @class  SubRegionMatching
 *  @brief  �̈�n��̌��菈���Ɋւ���N���X
 */
class SubRegionMatching
{
public:
  SubRegionMatching(const int start, const int end, const float norm, const float w_threshold,
    const float w_region = 1.0, const float w_rgbd = 1.0, const float w_I = 1.0,
    const float w_D = 3.0, const float w_F = 0.5);
  ~SubRegionMatching(void);
  //TemporalSubRegion subRegionMatching(vector<Features> &f_vec);

  RegionSequences streamingSubRegionMatching(vector<Features> &f_vec,
    vector<Mat> labels, const int start, const int end, Features &result);
  RegionSequences removeSequences(RegionSequences region_seq, const int min_length, 
    const float min_move);

  float diffSequence(TemporalVertex &v1, TemporalVertex &v2, RegionSequence region, 
    vector<Mat> labels, int N);
  RegionSequences sequenceSegmentation(RegionSequence seq, vector<Mat> labels,
    int N, float threshold);

  /////////////////////////////////////////////////////////////////////////////
  //void bidirectionalSubRegionMatching(TemporalSubRegion &fregion, 
  //  TemporalSubRegion &bregion, vector<Features> &f_vec, vector<Mat> labels);
  //RegionSequences regionToSequence2(TemporalSubRegion &fregion, TemporalSubRegion &bregion,
  //  vector<Features> &f_vec, const int min_frame, const float min_move);
  /////////////////////////////////////////////////////////////////////////////

private:
  void edgeConnection(vector<TemporalEdge> &edges, Feature &f, vector<Features> &f_vec, 
    const bool time);

  bool findTemporalEdge(vector<TemporalEdge> &edges, TemporalEdge &e);
  bool findTemporalEdge(vector<TemporalEdge> &edges, TemporalEdge &e, const bool time);
  float weightTemporalEdge(Feature &prev, Feature &next);

  void overlapSubRegionConnection(vector<TemporalEdge> &edges, Feature &f, vector<Features> &f_vec,
    const bool time, vector<Mat> &labels);
  float weightOverlagEdge(Feature &prev, Feature &next, int overlap);

  RegionSequences region2Sequences(TemporalRegion &region, vector<Features> &f_vec,
    vector<TemporalEdge> index = vector<TemporalEdge>());

  float diffSubRegion(Feature f1, Feature f2, Mat label, int N, int t);

  void internalMatching(TemporalRegion &region, const vector<Features> &f_vec, 
    vector<TemporalVertex> v, int frame, const bool time);

  /////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void bidirectionalEdgeConnection(vector<TemporalEdge> &f_edges, vector<TemporalEdge> &b_edges,
    Feature &f, vector<Features> &f_vec, const bool time, vector<Mat> &labels);
#endif
  /////////////////////////////////////////////////////////////////////////////


  int start;    /*!< �擪�t���[���ԍ� */
  int end;      /*!< �I���t���[���ԍ� */
  float norm_threshold;   /*!< �ړ��ʂ�臒l */
  float w_threshold; /*!< �d�݂�臒l */
  float w_region, w_rgbd; /*!< �摜�E�[�x�E�^�����̏d�݌W�� */
  float w_I, w_D, w_F;  /*!< �e�����̏d�݌W�� */
};

#endif