/*****************************************************************************/
/*! @addtogroup 
 *  @file   Misc.cpp
 *  @brief  �\���̂Ȃǂ��܂Ƃ߂����C�u�����Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "Misc.h"

//TemporalSubRegion::TemporalSubRegion(int frame_num, vector<int> elements,
//  vector<Features> &f_vec) 
//{
//  n_ranks.resize(frame_num);
//  p_ranks.resize(frame_num);
//  frames.resize(frame_num);
//  labels.resize(frame_num);
//  norms.resize(frame_num);
//  real_frame.resize(frame_num);
//
//  num = 0;
//
//  for (int i = 0; i < frame_num; i++) {
//    n_ranks.at(i).resize(elements.at(i));
//    p_ranks.at(i).resize(elements.at(i));
//    frames.at(i).resize(elements.at(i));
//    labels.at(i).resize(elements.at(i));
//    norms.at(i).resize(elements.at(i));
//    real_frame.at(i).resize(elements.at(i));
//  }
//
//  for (int y = 0; y < frame_num; y++) {
//    for (int x = 0; x < elements.at(y); x++) {
//      frames.at(y).at(x) = y;
//      labels.at(y).at(x) = x;
//      n_ranks.at(y).at(x) = 0;
//      p_ranks.at(y).at(x) = 0;
//      norms.at(y).at(x) = f_vec.at(y).at(x).norm3d;
//      real_frame.at(y).at(x) = f_vec.at(y).at(x).num;
//      num++;
//    }
//  }
//}
//
//TemporalSubRegion::~TemporalSubRegion(void)
//{
//}
//
//int TemporalSubRegion::find(int frame, int label, int &f, int &l, int &r_next, int &r_prev)
//{
//  int y = frame;
//  int x = label;
//  while(y != frames.at(y).at(x) || x != labels.at(y).at(x)) {    
//    int tmp = y;
//    y = frames.at(tmp).at(x);
//    x = labels.at(tmp).at(x);
//  }
//  frames.at(frame).at(label) = y;
//  labels.at(frame).at(label) = x;
//
//  real_frame.at(frame).at(label) = real_frame.at(y).at(x);
//
//  f = y;
//  l = x;
//  r_next = n_ranks.at(frame).at(label);
//  r_prev = p_ranks.at(frame).at(label);
//
//  return real_frame.at(y).at(x);
//}
//
//void TemporalSubRegion::join(int f_prev, int l_prev, int f_next, int l_next)
//{
//  frames.at(f_next).at(l_next) = f_prev;
//  labels.at(f_next).at(l_next) = l_prev;
//  n_ranks.at(f_next).at(l_next)++;
//  p_ranks.at(f_prev).at(l_prev)++;
//
//  real_frame.at(f_next).at(l_next) = real_frame.at(f_prev).at(l_prev);
//
//  float norm = max(norms.at(f_prev).at(l_prev), norms.at(f_next).at(l_next));
//  norms.at(f_prev).at(l_prev) = norm;
//  norms.at(f_next).at(l_next) = norm;
//
//  num--;
//}
//
//int TemporalSubRegion::num_sets(void) const { return num; };
//
//float TemporalSubRegion::getNorm(int frame, int label)
//{
//  int f = 0, l = 0, r_next = 0, r_prev = 0;
//  find(frame, label, f, l, r_next, r_prev);
//
//  return norms.at(f).at(l);
//}


/**
 * @brief �R���X�g���N�^
 * @param[in] frame_num   �t���[���̑���
 * @param[in] region_num  �e�t���[���ł̕����̈�̐�
 */
TemporalRegion::TemporalRegion(int frame_num, vector<int> region_num)
{
  num = 0;

  // �������̊m��
  elts.resize(frame_num);
  ranks.resize(frame_num);
  for (int i = 0; i < frame_num; i++) {
    elts.at(i).resize(region_num.at(i));
    ranks.at(i).resize(region_num.at(i));
  }

  // �e�v�f�̏�����
  for (int i = 0; i < frame_num; i++) {
    for (int j = 0; j < region_num.at(i); j++) {
      elts.at(i).at(j) = TemporalVertex(i, j);
      ranks.at(i).at(j) = 0;
      num++;
    }
  }
}

/**
 * @brief �����̈�̑Ή��t�����ʂ̒T��
 * @param[in] x �Ώۂ̕����̈�
 * @return  �Ή��t�����������̈�
 */
TemporalVertex TemporalRegion::find(TemporalVertex x)
{
  TemporalVertex y = x;
  while(!(y == elts.at(y.frame).at(y.label))) {
    y = elts.at(y.frame).at(y.label);
  }

  elts.at(x.frame).at(x.label) = y;
  return y;
}

/**
 * @brief �����̈�̑Ή��t�����ʂ̃����N�̒T��
 * @param[in] x �Ώۂ̕����̈�
 * @return  �Ή��t���̃����N
 */
int TemporalRegion::findRank(TemporalVertex x)
{
  TemporalVertex y = x;
  while(y != elts.at(y.frame).at(y.label)) {
    y = elts.at(y.frame).at(y.label);
  }

  elts.at(x.frame).at(x.label) = y;
  return ranks.at(y.frame).at(y.label);
}

/**
 * @brief �����̈�̑Ή��t��
 * @param[in] x �Ή��t���̑Ώۂ̕����̈�
 * @param[in] y �Ή��t���̑Ώۂ̕����̈�
 */
void TemporalRegion::join(TemporalVertex x, TemporalVertex y)
{
  if (ranks.at(x.frame).at(x.label) > ranks.at(y.frame).at(y.label)) {
    elts.at(y.frame).at(y.label) = x;
  }
  else {
    elts.at(x.frame).at(x.label) = y;
    if (ranks.at(x.frame).at(x.label) == ranks.at(y.frame).at(y.label)) {
      ranks.at(y.frame).at(y.label)++;
    }
  }
  num--;
}

/**
 * @brief �����̈�̋N�_��ݒ肵�Ή��t��
 * @param[in] root  �Ή��t���̑Ώۂ̕����̈�i�N�_�j
 * @param[in] v     �Ή��t���̑Ώۂ̕����̈�
 */
void TemporalRegion::joinRoot(TemporalVertex root, TemporalVertex v)
{
  elts.at(v.frame).at(v.label) = root;
  ranks.at(v.frame).at(v.label)++;
  ranks.at(root.frame).at(root.label)++;
  num--;
}

//void TemporalRegion::setRootLabel(TemporalVertex index, TemporalVertex root)
//{
//  elts.at(index.frame).at(index.label) = root;
//}

/**
 * @brief �̈�n�񐔂�Ԃ��i�ړ����Ȃ��̈�n����܂ށj
 */
int TemporalRegion::regionNum() const { return num; }



///* �����̈�̌n��̓��� */
//FeatureSequence::FeatureSequence(int frame, int label, int length)
//{
//  this->frame = frame;
//  this->label = label;
//  this->length = length;
//  features.resize(length);
//  labels.resize(length);
//  //    element.resize(length);
//}
//
//void FeatureSequence::assign(int num, Feature f) {
//  labels.at(num).push_back(f.label);
//  features.at(num) = f + features.at(num);
//  //    element.at(num)++;
//};
//
//void FeatureSequence::average(int num) {
//  int n = labels.at(num).size();
//  if (n != 0) {
//    features.at(num) = features.at(num) / n;
//  }
//};
//
//int FeatureSequence::getFrame(void) const { return frame; }
//int FeatureSequence::getLabel(void) const { return label; }
//int FeatureSequence::getLength(void) const { return length; }
////  void setUp(void) { centroid.x /= element_num; centroid.y /= element_num; };
////  Point2f location(void) const { return centroid; };
//Features FeatureSequence::getFeatures(void) const { return features; }
//Feature FeatureSequence::getFeature(int num) const { return features.at(num); }
////  int getElement(int num) const {return element.at(num); };
//vector<int> FeatureSequence::getLabels(int num) const { return labels.at(num); }
//
//// �����̈�n��̒������i�[
//void FeatureSequence::setLength(int length)
//{
//  this->length = length;
//}


// �̈�n��̓�����
/**
 * @brief �R���X�g���N�^
 * @param[in] label       �̈�n��̃��x��
 * @param[in] max_length  �����̍ő�t���[����
 */
RegionSequence::RegionSequence(TemporalVertex label, int max_length)
{
  this->label = label;
  this->length = 0;
  this->segment = true;
  this->norm = 0.0;
  features_vec.resize(max_length);
}

/**
 * @brief �̈�n��̃��x���̐ݒ�
 * @param[in] label �̈�n��̃��x��
 */
void RegionSequence::setLabel(TemporalVertex label) { this->label = label; }

/**
 * @brief �̈�n��̃t���[�����̐ݒ�
 * @param[in] length �̈�n��̃t���[����
 */
void RegionSequence::setLength(int length) { this->length = length; }

/**
 * @brief �̈�n��̓����ʂ̒ǉ�
 * @param[in] frame_num �Ώۂ̃t���[���ԍ�
 * @param[in] f         �̈�n��ɒǉ�����t���[���ԍ��̕����̈�
 */
void RegionSequence::setFeature(int frame_num, Feature f)
{
  Features &pf = features_vec.at(frame_num);

  if (pf.size() == 0) {
    length++;
  }

  pf.push_back(f);
}

/**
 * @brief �̈�n��̃��x���̎擾
 * @return  �̈�n��̃��x��
 */
TemporalVertex RegionSequence::getLabel(void) const { return label; }

/**
 * @brief �̈�n��̃t���[�����̎擾
 * @return  �̈�n��̃t���[����
 */
int RegionSequence::getLength(void) const { return length; }

/**
 * @brief �̈�n��̈ړ��ʂ̎擾
 * @return  �̈�n��̈ړ���
 */
float RegionSequence::getNorm(void) const { return norm; }

/**
 * @brief �̈�n��̓����ʂ̎擾
 * @param[in] frame_num �t���[���ԍ�
 * @return  �̈�n���frame_num�̓����ʁi�����̈�̔z��j
 */
Features RegionSequence::getFeatures(int frame_num) const
{ 
  return features_vec.at(frame_num); 
}

/**
 * @brief �̈�n��̈ړ��ʂ̕��ς��v�Z�i�����̈�̃V�[���t���[�̉��d���ρj
 */
void RegionSequence::calcNorm(void)
{
  int max_length = features_vec.size();
  for (int i = 0; i < max_length; i++) {
    Features &pf = features_vec.at(i);
    
    // �̈�̑���f�����v�Z
    int size = 0;
    for (int j = 0; j < pf.size(); j++) {
      size += pf.at(j).size;
    }

    // �V�[���t���[�̕��ς��v�Z
    for (int j = 0; j < pf.size(); j++) {
      norm += (pf.at(j).size * pf.at(j).norm3d) / size;
    }
  }

  // �̈�n��̃V�[���t���[�̕��ς��v�Z
  norm /= length;
}

/**
 * @brief �̈�n��̕������I��
 */
void RegionSequence::terminateSegment(void)
{
  segment = false;
}

/**
 * @brief �̈�n��̘a
 */
RegionSequence RegionSequence::operator + (RegionSequence seq)
{
  int max_length = this->features_vec.size();
  RegionSequence tmp_seq = *this;
  TemporalVertex root = tmp_seq.label;
  for (int i = 0; i < max_length; i++) {
    Features f1 = tmp_seq.getFeatures(i);
    Features f2 = seq.getFeatures(i);
    for (int j = 0; j < f2.size(); j++) {
      bool flag = false;
      for (int k = 0; k < f1.size(); k++) {
        if (f1.at(k).num == f2.at(j).num && f1.at(k).label == f2.at(j).label) {
          flag = true;
        }
      }
      if (!flag) {
        f2.at(j).root = root;
        tmp_seq.setFeature(i, f2.at(j));
      }
    }
  }

  return tmp_seq;
}

/**
 * @brief �̈�n��̐�
 */
RegionSequence RegionSequence::operator * (RegionSequence seq)
{
  int max_length = this->features_vec.size();
  RegionSequence tmp_seq(this->label, max_length);
  for (int i = 0; i < max_length; i++) {
    Features f1 = tmp_seq.getFeatures(i);
    Features f2 = seq.getFeatures(i);
    for (int j = 0; j < f2.size(); j++) {
      for (int k = 0; k < f1.size(); k++) {
        if (f1.at(k).num == f2.at(j).num && f1.at(k).label == f2.at(j).label) {
          tmp_seq.setFeature(i, f1.at(k));
        }
      }
    }
  }

  return tmp_seq;
}

// �O���[�o���֐�
/**
 * @brief �i�d�S�j�����̌v�Z
 * @param[in] p �O�t���[���̏d�S�̉�f
 * @param[in] n ��t���[���̏d�S�̉�f
 * @param[in] flow  �O�t���[���̃I�v�e�B�J���t���[
 * @return �i�d�S�j����
 */
float distanceCentroid(const Point2f &p, const Point2f &n, 
  const Point2f &flow)
{
  float dist_centroid = sqrtf((square((p.x + flow.x) - n.x)) 
    + (square((p.y + flow.y) - n.y)));
  return dist_centroid;
};

/**
 * @brief 3�����́i�d�S�j�����̌v�Z
 * @param[in] p �O�t���[���̏d�S��3�����ʒu
 * @param[in] n ��t���[���̏d�S��3�����ʒu
 * @param[in] flow  �O�t���[���̃V�[���t���[
 * @return 3�����́i�d�S�j����
 */
float distanceCentroid3d(const Point3f &p, const Point3f &n,
  const Point3f &flow)
{
  return sqrtf((square((p.x + flow.x) - n.x))
    + (square((p.y + flow.y) - n.y)) + (square((p.z + flow.z) - n.z)));
};

/**
* @brief pixel��mm�ϊ�
* @param[in]  pixel_x x�̉�f�ʒu
* @param[in]  pixel_y y�̉�f�ʒu
* @param[in]  depth   ��f�̐[�x�̒l
* @param[in]  width   ����
* @param[in]  height  �c��
* @return  �ϊ���̒l
*/
Point3f depth2World(const float pixel_x, const float pixel_y,
  const float depth, const int width, const int height)
{
  float	W;  // �[�x��depth�̂Ƃ��̍ő吅������(mm)
  float	H;  // �[�x��depth�̂Ƃ��̍ő吂������(mm)
  Point	P;  // �摜����B���摜�̍ő�pixel
  Point	p;  // �摜���S������W�_�܂ł�pixel
  float	w;  // ���S������W�_�܂ł̐�������(mm)
  float	h;  // ���S������W�_�܂ł̐�������(mm)
  Point3f	pt;  // �����E�ł̎O�������W

  // Kinect������W�_�܂ł̕�(mm)�����߂鎮
  W = (float)tan(THETA_X) * depth;
  P.x = width / 2;
  p.x = pixel_x - P.x;
  w = p.x * W / P.x;

  // Kinect������W�_�܂ł̍���(mm)�����߂鎮
  H = (float)tan(THETA_Y) * depth;
  P.y = height / 2;
  p.y = P.y - pixel_y;
  h = p.y * H / P.y;

  // �l����
  pt.x = w;
  pt.y = h;
  pt.z = depth;

  return pt;
}

/**
* @brief �V�[���t���[��pixel��mm�֕ϊ�
* @param[in,out] flow �ϊ�����t���[�̍s��i��f�{�P�x��mm�j
* @param[in]  d_curr  ���݂̐[�x�摜
* @param[in]  d_next  ���̐[�x�摜
* @param[in]  max_depth �����̍ő�v���l�imm�j
*/
void transformSceneFlow3d(Mat &flow, const Mat &d_curr,
  const Mat &d_next, const float max_depth)
{
  int width = flow.cols;
  int height = flow.rows;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Point3f &delta = flow.at<Point3f>(y, x);

      // �[�x����f�l(8bit)����mm�֕ϊ�
      float depth = (float)d_curr.at<uchar>(y, x) / 255.0 * max_depth;
      Point3f pt_prev = depth2World(x, y, depth, width, height);

      Point trans(x + (int)delta.x, y + (int)delta.y);
      if (trans.x < 0 || trans.x >= width || trans.y < 0 || trans.y >= height) {
        delta = Point3f(0, 0, 0);
        continue;
      }
      depth = (float)d_next.at<uchar>(trans) / 255.0 * max_depth;
      Point3f pt_next = depth2World(x, y, depth, width, height);

      // �V�[���t���[�̍X�V
      delta.x = pt_next.x - pt_prev.x;
      delta.y = pt_next.y - pt_prev.y;
      delta.z = pt_next.z - pt_prev.z;
    }
  }
}

/**
 * @brief �[�x�̕ϊ��i�P�x��mm�j
 * @param[in] d �[�x�i�P�x�j
 * @return  �[�x�imm�j
 */
float luminance2Depth(int d)
{
  return d / MAX_LUM * MAX_DEPTH;
}

/**
 * @brief �t���[��t�ł̗̈�n��̏W���ɑ����镔���̈�̎擾
 * @param[in] sequences �̈�n��̔z��
 * @param[in] t �t���[���ԍ�
 * @return �t���[��t�ł̗̈�n��̏W���ɑ����镔���̈�
 */
Features getFrameFeatures(RegionSequences sequences, int t)
{
  Features f;
  for (int i = 0; i < sequences.size(); i++) {
    Features &pf = sequences.at(i).getFeatures(t);
    for (int j = 0; j < pf.size(); j++) {
      pf.at(j).index = i; // �̈�n��ԍ����i�[
      f.push_back(pf.at(j));
    }
//    f.insert(f.end(), pf.begin(), pf.end());
  }

  return f;
}

/**
 * @brief �����̈�����x���̏����Ń\�[�g����֐�
 */
bool featuresLabelAsc(const Feature &a, const Feature &b)
{
  return a.num == b.num ? a.label < b.label : a.num < b.num;
}