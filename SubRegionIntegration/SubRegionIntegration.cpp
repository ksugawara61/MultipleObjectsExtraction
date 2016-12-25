/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionIntegration.cpp
 *  @brief  �̈�n��̓��������Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "SubRegionIntegration.h"

// �����̈�̌n��̓���
/**
 * @brief �R���X�g���N�^
 * @param[in] threshold �̈�n��̓�����臒l
 * @param[in] w_locate  3�����ʒu�̏d�݌W��
 * @param[in] w_flow    �V�[���t���[�̏d�݌W��
 * @param[in] width     �t���[���̉���
 * @param[in] height    �t���[���̏c��
 * @param[in] start     �擪�t���[���ԍ�
 * @param[in] end       �����t���[���ԍ�
 * @param[in] near_flag (1:�ŋߓ_�΂̃V�[���t���[�𗘗p 0:�����̈斈�̃V�[���t���[�𗘗p�j
 * @param[in] min_contact �̈�n��Ԃ̐ڐG�񐔂�臒l(0:����)
 */
SubRegionIntegration::SubRegionIntegration(const float threshold, const float w_locate,
  const float w_flow, const int width, const int height, 
  const int start, const int end, bool near_flag, const int min_contact)
{
  this->threshold = threshold;
  this->min_contact = min_contact;
  this->w_locate = w_locate;
  this->w_flow = w_flow;
  this->width = width;
  this->height = height;
  this->start = start;
  this->end = end;
  this->near_flag = near_flag;
}

/**
 * @brief �f�X�g���N�^
 */
SubRegionIntegration::~SubRegionIntegration(void)
{
}

/**
 * @brief �̈�n��̓���
 * @param[in] sequences �̈�n��̃��X�g
 * @param[in] flows     �V�[���t���[�̃��X�g
 * @return �̈�n��̓�������
 */
RegionSequences SubRegionIntegration::integrateSequence(RegionSequences sequences,
  const vector<Mat> &flows)
{
  // �O���t�̍\�z
  vector<edge> edges;
  int num_vertex = sequences.size();
  for (int y = 0; y < num_vertex; y++) {
    for (int x = y + 1; x < num_vertex; x++) {
      float distance = sequencesDistance(flows, sequences, x, y);

      cout << y << " " << x << " : " << distance << endl;
      if (distance <= threshold && distance != -1) {
        // �G�b�W�̒ǉ�
        edge e;
        e.a = x;
        e.b = y;
        e.w = 0;
        edges.push_back(e);
      }
    }
  }

  // �̈�n��̓���
  int num_edges = edges.size();
  SubRegion index(num_vertex, 1);
  for (int i = 0; i < num_edges; i++) {
    edge &pedge = edges.at(i);

    int a = index.find(pedge.a);
    int b = index.find(pedge.b);

    if (a != b) {
      if (pedge.w < 1) {
        index.join(a, b);
      }
    }
  }

  // �̈�n��̓������ʂ�SubRegion����RegionSequences�֕ϊ�
  vector<int> count(num_vertex);
  for (int i = 0; i < num_vertex; i++) {
    int j = index.find(i);
    sequences.at(j) = sequences.at(j) + sequences.at(i);
    count.at(j)++;
  }

  RegionSequences tmp_seq;
  for (int i = 0; i < num_vertex; i++) {
    if (count.at(i) != 0) {
      tmp_seq.push_back(sequences.at(i));
    }
  }

  // �ڐG�E�������s��Ȃ��ΏۊO�̗̈�n��̏���
  if (min_contact != 0) {
    num_vertex = tmp_seq.size();
    vector<int> flag(num_vertex);
    for (int i = 0; i < num_vertex; i++) {
      for (int j = i + 1; j < num_vertex; j++) {
        //      cout << i << " " << j << " " << " : " << contactDetection(tmp_seq.at(i), tmp_seq.at(j), flows) << endl;
        if (contactDetection(tmp_seq.at(i), tmp_seq.at(j), flows)) {
          flag.at(i) = flag.at(j) = 1;
        }
      }
    }
    for (int i = 0; i < num_vertex; i++) {
      if (!(flag.at(i))) {
        tmp_seq.erase(tmp_seq.begin() + i);
        flag.erase(flag.begin() + i);
        num_vertex--;
        i--;
      }
    }
  }

  return tmp_seq;
}

/**
 * @brief �̈�n��Ԃ̑���x���v�Z
 * @param[in] flows �V�[���t���[�̃��X�g
 * @param[in] s     �̈�n��̃��X�g
 * @param[in] a     �̈�n��A
 * @param[in] b     �̈�n��B
 * @return  �̈�n��Ԃ̑���x
 */
float SubRegionIntegration::sequencesDistance(const vector<Mat> &flows, 
  const vector<RegionSequence> &s, const int a, const int b)
{
  int length = end - start;
  float dist = 0.0;

  int n = 0;  // �̈�n��̏d�Ȃ鐔
  for (int t = 0; t < length; t++) {
    if (s.at(a).getFeatures(t).size() != 0 && s.at(b).getFeatures(t).size() != 0) {
      vector<Point> pt1, pt2;
      readDistancePointDat(t + start, a, b, pt1, pt2, start, end - 1);

      if (near_flag) {
        dist += subregionsDistance(flows.at(t), pt1, pt2, t + start);
      }
      else {
        dist += subregionBasedDistance(pt1, pt2, s.at(a).getFeatures(t),
          s.at(b).getFeatures(t), t + start);
      }
      n++;
    }
  }

  // ����x�̕��ϒl���v�Z
  if (n > 0) {
    dist /= n;
  }
  else {
    dist = -1;
  }

  return dist;
}


/**
 * @brief �����̈�Ԃ̑���x���v�Z
 * @param[in] flow  �V�[���t���[
 * @param[in] pt1   �����̈�At�̉�f
 * @param[in] pt2   �����̈�Bt�̉�f
 * @param[in] t   �t���[���ԍ�
 * @return  �����̈�At��Bt�Ԃ̑���x
 */
float SubRegionIntegration::subregionsDistance(const Mat &flow, 
  const vector<Point> &pt1, const vector<Point> &pt2, const int t)
{
  // �ϐ��錾
  int N = pt1.size();
  Mat depth;
  char name[80];
  vector<Point3f> locate1_list(N), flow1_list(N);
  vector<Point3f> locate2_list(N), flow2_list(N);
  float dist_locate = 0.0, dist_flow = 0.0;
  float dist = 0.0;
  
  // �[�x���̓ǂݍ���
  sprintf(name, "./data/depth-0%004d.bmp", t);
  depth = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
  
  // �R�����̋�Ԉʒu�Ɓi�R�����́j�V�[���t���[�����X�g�Ɋi�[
  for (int i = 0; i < N; i++) {
    Point p = pt1.at(i);
    float d = luminance2Depth(depth.at<uchar>(p));
    locate1_list.at(i) = depth2World(p.x, p.y, d, width, height);
    flow1_list.at(i) = flow.at<Point3f>(p);

    p = pt2.at(i);
    d = luminance2Depth(depth.at<uchar>(p));
    locate2_list.at(i) = depth2World(p.x, p.y, d, width, height);
    flow2_list.at(i) = flow.at<Point3f>(p);
  }

  //// �R�����ʒu�ł̋����E�V�[���t���[�̋������v�Z�i���ρj
  //for (int i = 0; i < N; i++) {
  //  dist_locate += distanceCentroid3d(locate1_list.at(i), locate2_list.at(i));
  //  dist_flow += distanceCentroid3d(flow1_list.at(i), flow2_list.at(i));
  //}
  //dist = (dist_locate / N) * w_locate + (dist_flow / N) * w_flow; 

  // �R�����ʒu�ł̋����E�V�[���t���[�̋������v�Z�i�����l�j
  vector<float> dist_list;
  for (int i = 0; i < N; i++) {
    dist_locate = distanceCentroid3d(locate1_list.at(i), locate2_list.at(i));
    dist_flow = distanceCentroid3d(flow1_list.at(i), flow2_list.at(i));

    dist_list.push_back(dist_locate * w_locate + dist_flow * w_flow);
  }

  // ����x�̃\�[�g
  sort(dist_list.begin(), dist_list.end());
  
  // �����l�̌���
  if (N % 2 != 0) {
    dist = dist_list.at(cvFloor(N / 2.0));
  }
  else {
    dist = (dist_list.at(N / 2) + dist_list.at(N / 2 - 1)) / 2.0;
  }

  return dist;
}

/**
 * @brief �����̈�Ԃ̑���x���v�Z�i�����̈�̃V�[���t���[�Ɋ�Â��v�Z�j
 * @param[in] pt1   �����̈�At�̉�f
 * @param[in] pt2   �����̈�Bt�̉�f
 * @param[in] f1    �����̈�At�̓�����
 * @param[in] f2    �����̈�Bt�̓�����
 * @param[in] t   �t���[���ԍ�
 * @return  �����̈�At��Bt�Ԃ̑���x
 */
float SubRegionIntegration::subregionBasedDistance(const vector<Point> &pt1, 
  const vector<Point> &pt2, const Features f1, const Features f2, const int t)
{
  // �ϐ��錾
  int N = pt1.size();
  Mat depth;
  char name[80];
  vector<Point3f> locate1_list(N), locate2_list(N);
  float dist_locate = 0.0, dist_flow = 0.0;
  float dist = 0.0;
  
  // �[�x���̓ǂݍ���
  sprintf(name, "./data/depth-0%004d.bmp", t);
  depth = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
  
  // �R�����̋�Ԉʒu�Ɓi�R�����́j�V�[���t���[�����X�g�Ɋi�[
  for (int i = 0; i < N; i++) {
    Point p = pt1.at(i);
    float d = luminance2Depth(depth.at<uchar>(p));
    locate1_list.at(i) = depth2World(p.x, p.y, d, width, height);

    p = pt2.at(i);
    d = luminance2Depth(depth.at<uchar>(p));
    locate2_list.at(i) = depth2World(p.x, p.y, d, width, height);
  }

  //// �R�����ʒu�ł̋����E�V�[���t���[�̋������v�Z�i���ϒl�j
  //for (int i = 0; i < N; i++) {
  //  dist_locate += distanceCentroid3d(locate1_list.at(i), locate2_list.at(i));
  //  dist_flow += distanceCentroid3d(f1, f2);
  //}
  //dist = (dist_locate / N) * w_locate + (dist_flow / N) * w_flow; 

  // �R�����ʒu�ł̋����E�V�[���t���[�̋������v�Z�i�����l�̕��ρj
  // �̈�T�C�Y�����Ƃɉ��d���ςŃV�[���t���[���v�Z
  Point3f s1, s2;
  int size = 0;
  /*for (int i = 0; i < f1.size(); i++) {
    size += f1.at(i).size;
  }
  for (int i = 0; i < f1.size(); i++) {
    s1.x += f1.at(i).flow3d.x * f1.at(i).size / size;
    s1.y += f1.at(i).flow3d.y * f1.at(i).size / size;
    s1.z += f1.at(i).flow3d.z * f1.at(i).size / size;
  }*/

  // �V�[���t���[�̑�����̈搔�Ŋ���
  for (int i = 0; i < f1.size(); i++) {
    s1 += f1.at(i).flow3d;
    size++;
  }
  s1.x /= size;
  s1.y /= size;
  s1.z /= size;

  size = 0;
  /*for (int i = 0; i < f2.size(); i++) {
    size += f2.at(i).size;
  }
  for (int i = 0; i < f2.size(); i++) {
    s2.x += f2.at(i).flow3d.x * f2.at(i).size / size;
    s2.y += f2.at(i).flow3d.y * f2.at(i).size / size;
    s2.z += f2.at(i).flow3d.z * f2.at(i).size / size;
  }*/

  // �V�[���t���[�̑�����̈搔�Ŋ���
  for (int i = 0; i < f2.size(); i++) {
    s2 += f2.at(i).flow3d;
    size++;
  }
  s2.x /= size;
  s2.y /= size;
  s2.z /= size;

  vector<float> dist_list;
  for (int i = 0; i < N; i++) {
    dist_locate = distanceCentroid3d(locate1_list.at(i), locate2_list.at(i));
    dist_flow = distanceCentroid3d(s1, s2);
    dist_list.push_back(dist_locate * w_locate + dist_flow * w_flow);
  }

  // ����x�̃\�[�g
  sort(dist_list.begin(), dist_list.end());
  
  // �����l�̌���
  if (dist_list.size() == 0) {
  }

  if (N % 2 != 0) {
    dist = dist_list.at(cvFloor(N / 2.0));
  }
  else {
    dist = (dist_list.at(cvFloor(N / 2.0)) + dist_list.at(cvFloor(N / 2.0) - 1)) / 2.0;
  }

  return dist;
}

/**
 * @brief �̈�n��Ԃ̐ڐG�֌W�̌��o
 * @param[in] seqA  �̈�n��A
 * @param[in] seqB  �̈�n��B
 * @param[in] flows �V�[���t���[
 * @return  true  �ڐG�L��
 *          false �ڐG����
 */
bool SubRegionIntegration::contactDetection(RegionSequence seqA, RegionSequence seqB, const vector<Mat> &flows)
{
  // �ϐ��錾
  float min_dist = FLT_MAX;
  int length = end - start;
  int count = 0;  // �ڐG�����t���[����

  // �̈�n��Ԃ��ŒZ�����̃t���[�����v�Z
  for (int t = 0; t < length; t++) {
    // �t���[��t�ł̗̈�n��A, B�ɑ������f��T��
    Features At = seqA.getFeatures(t);
    Features Bt = seqB.getFeatures(t);

    min_dist = FLT_MAX;
    for (int i = 0; i < At.size(); i++) {
      for (int j = 0; j < Bt.size(); j++) {
        vector<Point> pt1, pt2;
        readDistancePointDat(t + start, At.at(i).index, Bt.at(j).index, pt1, pt2, start, end - 1);

        if (near_flag) {
          min_dist = min(min_dist, subregionsDistance(flows.at(t), pt1, pt2, t + start));
        }
        else {
          min_dist = min(min_dist, subregionBasedDistance(pt1, pt2, At, Bt, t + start));
        }
      }
    }

    if (min_dist < threshold) {
      count++;
    }
  }

  cout << "contact frame num : " << count << endl;

  return count >= min_contact; //min_dist < threshold;
}


///**
// * @brief �����̈�n��̗ގ��x���v�Z(-1�`1)
// * @param[in] x �����̈�n��x
// * @param[in] y �����̈�n��y
// * @return  �����̈�n��xy�Ԃ̗ވȓx�i�R�T�C���ވȓx�̕��ρj
// */
//float SubRegionIntegration::sequenceCosineSimilarity(const vector<Mat> &flows, 
//  const vector<FeatureSequence> &s, const int x, const int y)
//{
//  int length = s[x].getLength();
//  float similarity = 0.0;
//
//  int n = 0;  // �n��̏d�Ȃ鐔
//  for (int i = 0; i < length; i++) {
//    if (s[x].getLabels(i).size() != 0 && s[y].getLabels(i).size() != 0) {
//      vector<Point> pt1, pt2;
//      readDistancePointDat(i + start, x, y, pt1, pt2);
//
//      similarity += cosineSimilarity(flows.at(i), pt1, pt2, i + start);
//      n++;
//    }
//  }
//
//  // �ގ��x�̕��ς��v�Z
//  if (n > 0 && n > min_sequence) {
//    similarity /= n;
//  }
//  else {
//    similarity = -1;
//  }
//
//  return similarity;
//}
//
///**
// * @brief �����̈�Ԃ̗ގ��x���v�Z�i�R�T�C���ގ��x�j
// * @param[in] pt1 ���ډ�f�̃��X�g
// * @param[in] pt2 ���ډ�f�̃��X�g
// * @param[in] num �t���[���ԍ�
// * @return  �����̈�Ԃ́i�R�T�C���j�ވȓx
// */
//float SubRegionIntegration::cosineSimilarity(const Mat &flow, 
//  const vector<Point> &pt1, const vector<Point> &pt2, const int num)
//{
//  const int FEATURE_NUM = 6;
//
//  // �ϐ��錾
//  int iteration = pt1.size();
//  Mat depth;
//  char name[80];
//  vector<Point3f> locate1_list(iteration), flow1_list(iteration);
//  vector<Point3f> locate2_list(iteration), flow2_list(iteration);
//  Point3f locate1(0, 0, 0), locate2(0, 0, 0), flow1(0, 0, 0), flow2(0, 0, 0);
//  vector<float> f1(FEATURE_NUM), f2(FEATURE_NUM);
//  float v12 = 0.0, v1 = 0.0, v2 = 0.0;
//
//  // �[�x���ƃV�[���t���[�̓ǂݍ���
//  sprintf(name, "./data/depth-0%004d.bmp", num);
//  depth = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
//  //flow = readSceneFlowDat(num, width, height);
//
//  // �R�����̋�Ԉʒu�ƃV�[���t���[�����X�g�Ɋi�[
//  for (int i = 0; i < iteration; i++) {
//    Point p = pt1.at(i);
//    float d = depth.at<uchar>(p);
//    locate1_list.at(i) = depth2World(p.x, p.y, d, width, height);
//    flow1_list.at(i) = flow.at<Point3f>(p);
//
//    p = pt2.at(i);
//    d = depth.at<uchar>(p);
//    locate2_list.at(i) = depth2World(p.x, p.y, d, width, height);
//    flow2_list.at(i) = flow.at<Point3f>(p);
//  }
//
//  // �e�����i�R�����ʒu�E�V�[���t���[�j�̕��ϒl���v�Z
//  for (int i = 0; i < iteration; i++) {
//    locate1.x += locate1_list.at(i).x;
//    locate1.y += locate1_list.at(i).y;
//    locate1.z += locate1_list.at(i).z - (MAX_DEPTH / 2); // ���K���Ń}�C�i�X�l���܂߂邽��
//
//    locate2.x += locate2_list.at(i).x;
//    locate2.y += locate2_list.at(i).y;
//    locate2.z += locate2_list.at(i).z - (MAX_DEPTH / 2); // ���K���Ń}�C�i�X�l���܂߂邽��
//
//    flow1.x += flow1_list.at(i).x;
//    flow1.y += flow1_list.at(i).y;
//    flow1.z += flow1_list.at(i).z;
//
//    flow2.x += flow2_list.at(i).x;
//    flow2.y += flow2_list.at(i).y;
//    flow2.z += flow2_list.at(i).z;
//  }
//
//  locate1.x /= iteration;
//  locate1.y /= iteration;
//  locate1.z /= iteration;
//
//  locate2.x /= iteration;
//  locate2.y /= iteration;
//  locate2.z /= iteration;
//
//  flow1.x /= iteration;
//  flow1.y /= iteration;
//  flow1.z /= iteration;
//
//  flow2.x /= iteration;
//  flow2.y /= iteration;
//  flow2.z /= iteration;
//
//  // �e�����̐��K��
//  f1.at(0) = locate1.x / sqrtf(square(locate1.x) + square(locate2.x));
//  f1.at(1) = locate1.y / sqrtf(square(locate1.y) + square(locate2.y));
//  f1.at(2) = locate1.z / sqrtf(square(locate1.z) + square(locate2.z));
//  f1.at(3) = flow1.x / sqrtf(square(flow1.x) + square(flow2.x));
//  f1.at(4) = flow1.y / sqrtf(square(flow1.y) + square(flow2.y));
//  
//  float dist = sqrtf(square(flow1.z) + square(flow2.z));
//  if (dist != 0) {
//    f1.at(5) = flow1.z / dist;
//  }
//  else if (flow1.z == 0) {
//    f1.at(5) = 0.0;
//  }
//
//  f2.at(0) = locate2.x / sqrtf(square(locate1.x) + square(locate2.x));
//  f2.at(1) = locate2.y / sqrtf(square(locate1.y) + square(locate2.y));
//  f2.at(2) = locate2.z / sqrtf(square(locate1.z) + square(locate2.z));
//  f2.at(3) = flow2.x / sqrtf(square(flow1.x) + square(flow2.x));
//  f2.at(4) = flow2.y / sqrtf(square(flow1.y) + square(flow2.y));
//
//  dist = sqrtf(square(flow1.z) + square(flow2.z));
//  if (dist != 0) {
//    f2.at(5) = flow2.z / dist;
//  }
//  else {
//    f2.at(5) = 0.0;
//  }
//
//  // �����Ԃ̃R�T�C���ގ��x�̌v�Z
//  for (int i = 0; i < FEATURE_NUM; i++) {
//    v12 += f1.at(i) * f2.at(i);
//    v1 += square(f1.at(i));
//    v2 += square(f2.at(i));
//  }
//  v1 = sqrtf(v1);
//  v2 = sqrtf(v2);
//
//  return v12 / (v1 * v2);
//}



///**
// * @brief �����̈�Ԃ̋����̕��ϒl���v�Z
// * @param[in] distmat �����̈�Ԃ̋����̌n��
// * @param[in] x
// * @param[in] y
// * @return  �����̕��ϒl
// */
//float SubRegionIntegration::averageDistance(const vector<Mat> &distmat,
//  const int x, const int y)
//{
//  int frame_num = distmat.size();
//
//  float average = 0.0;
//
//  // �����̑������v�Z
//  int num = frame_num;
//  for (int i = 0; i < frame_num; i++) {
//    float tmp = distmat.at(i).at<float>(y, x);
//    if (tmp == 0.0) {
//      num--;
//      continue;
//    }
//
//    average += tmp;
//  }
//
//  // ���ϒl�̌v�Z
//  if (num > 0 && num > min_num) {
//    average /= num;
//  }
//  else {
//    average = -1;
//  }
//
//  return average;
//}
//
///**
// * @brief �����̈�Ԃ̋����̕W���΍����v�Z
// * @param[in] distmat �����̈�Ԃ̋����̌n��
// * @param[in] x
// * @param[in] y
// * @return  �����̕W���΍�
// */
//float SubRegionIntegration::stdevDistance(const vector<Mat> &distmat,
//  const int x, const int y)
//{
//  int frame_num = distmat.size();
//
//  float average = 0.0;
//
//  // �����̑������v�Z
//  vector<float> dist_vec; // �e�t���[���̋���
//  int num = frame_num;
//  for (int i = 0; i < frame_num; i++) {
//    float tmp = distmat.at(i).at<float>(y, x);
//    if (tmp == 0.0) {
//      num--;
//      continue;
//    }
//
//    average += tmp;
//    dist_vec.push_back(tmp);
//  }
//
//  // �W���΍��̌v�Z
//  float stdev = 0.0;
//  if (num > 0) {
//    average /= num;
//
//    // ���U�̌v�Z
//    for (int i = 0; i < num; i++) {
//      stdev += square(dist_vec.at(i) - average);
//    }
//    stdev /= num;
//
//    stdev = sqrtf(stdev);
//  }
//  else {
//    stdev = -1;
//  }
//
//  return stdev;
//}
//  
///**
// * @brief �d�݂̌v�Z
// * @param[in] s1  �����̈�̌n��P
// * @param[in] s2  �����̈�̌n��Q
// * @return �d��
// */
//float SubRegionIntegration::weightSequences(const FeatureSequence &s1, 
//  const FeatureSequence &s2)
//{
//  int length = s1.getLength();
//  float weight = 0.0;
//
//  int num = 0;
//  for (int i = 0; i < length; i++) {
//    if (s1.getLabels(i).size() != 0 && s2.getLabels(i).size() != 0) {
//      Feature f1 = s1.getFeature(i);
//      Feature f2 = s2.getFeature(i);
//
//      weight += sqrtf(square(f1.flow2d.x - f2.flow2d.x)
//        + square(f1.flow2d.y - f2.flow2d.y)
//        + square(f1.flow2d.z - f2.flow2d.z));
//      num++;
//    }
//  }
//
//  if (num > 0) {
//    weight /= num;
//  }
//  else {
//    weight = -1;
//  }
//
//  return weight;
//}


//// �ʒu�̕]����
//float SubRegionIntegration::sequenceLocation(const vector<FeatureSequence> &s,
//  const int x, const int y)
//{
//  int length = s[x].getLength();
//  float dist = 0.0;
//
//  int n = 0;  // �n��̏d�Ȃ鐔
//  for (int i = 0; i < length; i++) {
//    if (s[x].getLabels(i).size() != 0 && s[y].getLabels(i).size() != 0) {
//      vector<Point> pt1, pt2;
//      readDistancePointDat(i + start, x, y, pt1, pt2);
//
//      dist += subregionLocation(pt1, pt2, i + start);
//      n++;
//    }
//  }
//
//  // �ގ��x�̕��ς��v�Z
//  if (n > 0 && n > min_num) {
//    dist /= n;
//  }
//  else {
//    dist = -1;
//  }
//
//  return dist;
//}
//
//float SubRegionIntegration::subregionLocation(const vector<Point> &pt1, 
//  const vector<Point> &pt2, const int num)
//{
//  // �ϐ��錾
//  int iteration = pt1.size();
//  Mat depth;
//  char name[80];
//  vector<Point3f> list1(iteration), list2(iteration);
//  float dist = 0.0;
//  
//  // �[�x���ƃV�[���t���[�̓ǂݍ���
//  sprintf(name, "./data/depth-0%004d.bmp", num);
//  depth = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
//  
//  // �R�����̋�Ԉʒu�Ɓi�R�����́j�V�[���t���[�����X�g�Ɋi�[
//  for (int i = 0; i < iteration; i++) {
//    Point p = pt1.at(i);
//    float d = depth.at<uchar>(p) / 255.0 * MAX_DEPTH;
//    list1.at(i) = depth2World(p.x, p.y, d, width, height);
//
//    p = pt2.at(i);
//    d = depth.at<uchar>(p) / 255.0 * MAX_DEPTH;
//    list2.at(i) = depth2World(p.x, p.y, d, width, height);
//  }
//
//  // �R�����ʒu�ł̋����E�V�[���t���[�̋������v�Z
//  for (int i = 0; i < iteration; i++) {
//    dist += distanceCentroid3d(list1.at(i), list2.at(i));
//  }
//
//  return dist;
//}
//
//
//// �t���[�̕]����
//float SubRegionIntegration::sequenceSceneFlow(const vector<Mat> &flows, const vector<FeatureSequence> &s,
//  const int x, const int y)
//{
//  int length = s[x].getLength();
//  float dist = 0.0;
//
//  int n = 0;  // �n��̏d�Ȃ鐔
//  for (int i = 0; i < length; i++) {
//    if (s[x].getLabels(i).size() != 0 && s[y].getLabels(i).size() != 0) {
//      vector<Point> pt1, pt2;
//      readDistancePointDat(i + start, x, y, pt1, pt2);
//
//      dist += subregionSceneFlow(flows.at(i), pt1, pt2, i + start);
//      n++;
//    }
//  }
//
//  // �ގ��x�̕��ς��v�Z
//  if (n > 0 && n > min_num) {
//    dist /= n;
//  }
//  else {
//    dist = -1;
//  }
//
//  return dist;
//}
//
//float SubRegionIntegration::subregionSceneFlow(const Mat &flow, const vector<Point> &pt1, 
//  const vector<Point> &pt2, const int num)
//{
//  // �ϐ��錾
//  int iteration = pt1.size();
//  char name[80];
//  vector<Point3f> list1(iteration), list2(iteration);
//  float dist = 0.0;
//    
//  // �R�����̋�Ԉʒu�Ɓi�R�����́j�V�[���t���[�����X�g�Ɋi�[
//  for (int i = 0; i < iteration; i++) {
//    Point p = pt1.at(i);
//    list1.at(i) = flow.at<Point3f>(p);
//
//    p = pt2.at(i);
//    list2.at(i) = flow.at<Point3f>(p);
//  }
//
//  // �R�����ʒu�ł̋����E�V�[���t���[�̋������v�Z
//  for (int i = 0; i < iteration; i++) {
//    dist += distanceCentroid3d(list1.at(i), list2.at(i));
//  }
//
//  return dist;
//}