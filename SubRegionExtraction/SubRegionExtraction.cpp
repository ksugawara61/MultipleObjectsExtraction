/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionExtraction.cpp
 *  @brief  �����̈�̒��o�����Ɋւ���t�@�C��
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "SubRegionExtraction.h"

/**
 * @brief �R���X�g���N�^
 * @param[in] sigma     ��Ԃ̃K�E�X���z�̕W���΍�
 * @param[in] ap_filter �t�B���^�̑��T�C�Y
 * @param[in] k         �����̈�̓�����臒l
 * @param[in] min_size  �����̈�̍ŏ��T�C�Y
 * @param[in] w_I       �摜���̏d�݌W��
 * @param[in] w_D       �[�x���̏d�݌W��
 * @param[in] w_F       �^�����̏d�݌W��
 * @param[in] color_sigma �P�x�̃K�E�X���z�̕W���΍�
 * @param[in] smooth_it �������̔�����
 * @param[in] ap_flow   �t���[�̌v�Z�ɗp���鑋�T�C�Y
 * @param[in] levels    �t���[�̉摜�s���~�b�h�̃��x��
 * @param[in] iterations  �t���[�̌v�Z�A���S���Y���̔�����
 * @param[in] scale     �摜�s���~�b�h�̃X�P�[���T�C�Y
 */
SubRegionExtraction::SubRegionExtraction(const float sigma, const int ap_filter,
  const float k, const int min_size, const float w_I, const float w_D, const float w_F,
  const float color_sigma, const int smooth_it, 
  const int ap_flow, const int levels, const int iterations, const float scale)
{
  this->sigma = sigma;
  this->ap_filter = ap_filter;
  this->k = k;
  this->min_size = min_size;
  this->w_I = w_I;
  this->w_D = w_D;
  this->w_F = w_F;
  this->color_sigma = color_sigma;
  this->smooth_it = smooth_it;
  this->ap_flow = ap_flow;
  this->levels = levels;
  this->iterations = iterations;
  this->scale = scale;
  ap_boundary = ap_filter * 2;  // ��ŏC�������ق����ǂ�
}

/**
* @brief �f�X�g���N�^
*/
SubRegionExtraction::~SubRegionExtraction(void)
{
}

/**
* @brief �����̈�̒��o
* @param[out]  label �����̈�̃��x��
* @param[in] frame_num �t���[���ԍ�
* @param[in] c_curr  �摜���i�Ώۃt���[���j
* @param[in] d_curr  �[�x���i�Ώۃt���[���j
* @param[in] c_next  �摜���i���̃t���[���j
* @param[in] d_next  �[�x���i���̃t���[���j
* @param[out] boundarys  �����̈�̋��E�̃��X�g
* @return  �Ώۃt���[���̓�����
*/
Features SubRegionExtraction::subRegionExtract(Mat &label, const int frame_num,
  Mat c_curr, Mat d_curr, const Mat &c_next, const Mat &d_next,
  vector<Mat> &boundarys)
{
  // �V�[���t���[�̐���
  Mat flow;
  SceneFlow::sceneFlowFarneback(c_curr, d_curr, c_next, d_next, flow, scale,
    levels, ap_flow, iterations, ap_flow / pow((float)(1 / scale), (float)(levels - 1)), 1,1);
  //SceneFlow::sceneFlowLK(c_curr, d_curr, c_next, d_next, flow, Size(15, 15));
  //SceneFlow::sceneFlowSimple(c_curr, d_curr, c_next, d_next, flow);

  //outOpticalFlowDat(flow, frame_num, 3);
  //outSceneFlowDat(d_curr, flow, frame_num, 3);
  
  // �V�[���t���[��3�������ƃt�@�C���o��
  Mat flow3d = flow.clone();
  transformSceneFlow3d(flow3d, d_curr, d_next, MAX_DEPTH);
  FileOperation::writeSceneFlowDat(flow3d, frame_num);

  // �����̈�̒��o
  /*GraphBased gb(sigma, ap_filter, k, min_size, w_I, w_D, w_F,
    color_sigma, smooth_it);*/

  // ������
  Mat c_smooth = c_curr.clone();
  Mat d_smooth = d_curr.clone();
  smoothing(c_smooth, d_smooth);

  c_curr.convertTo(c_curr, CV_32FC3);
  d_curr.convertTo(d_curr, CV_32FC1);

  GraphBased gb(k, min_size, w_I, w_D, w_F);
  SubRegion region = gb.segmentImage(c_smooth, d_smooth, flow3d);

  // �����ʂ̌v�Z�̑O�����i�����̈�̋��E�ߖT�̃t���[�������j
  Mat boundary;
  label.release();
  int region_num = preprocess(label, boundary, region);

  // �����ʂ̌v�Z
  Features f_vec = calcFeatures(label, frame_num, region_num,
    c_curr, d_curr, flow, boundary);

  // ���E�̎擾
  boundarys.push_back(boundary);

  char name[80];
  string buf;

  // �`�揈��
  // �e�t���[���̕����̈��`��
  sprintf(name, "./out_subregion/subregion-0%004d.png", frame_num);
  buf = name;
  Mat draw = drawSubRegion(label);
//  Mat move = drawMovingSubRegion(label, f_vec, region, 10);
  //Mat opt_flow = drawOpticalFlow(c_curr, flow, boundary, 3, ap_flow);
  Mat average = drawSubRegionAverage(label, boundary, f_vec);
  Mat region_flow = drawSubRegionFeature(label, boundary, f_vec, 5, 10);
  imwrite(buf, draw);

  sprintf(name, "./out_subregion/subregionflow-0%004d.png", frame_num);
  buf = name;
  imwrite(buf, region_flow);

  sprintf(name, "./out_subregion/average-0%004d.png", frame_num);
  buf = name;
  imwrite(buf, average);

  sprintf(name, "./subregion/boundary-0%004d.png", frame_num);
  imwrite(name, drawBoundary(label, ap_boundary));

  imshow("subregion", draw);
  imshow("region feature", region_flow);
  waitKey(10);

  return f_vec;
}

//Features SubRegionExtraction::correctSubRegionExtract(Mat &label, const int frame_num,
//  const Mat &c_prev, const Mat &d_prev, const Mat &c_curr, const Mat &d_curr,
//  const Mat &c_next, const Mat &d_next, vector<Mat> &boundarys)
//{
//  // �ߋ��t���[���̃t���[�𐄒�
//  Mat flow;
//  SceneFlow::sceneFlowFarneback(c_prev, d_prev, c_curr, d_curr, flow, scale,
//    levels, ap_flow, iterations, ap_flow / pow((float)(1 / scale), (float)(levels - 1)), 1,1);
//
//  // �����̈�̒��o
//  TemporalGraphBased gb(sigma, ap_filter, k, min_size, w_I, w_D, 
//    color_sigma, smooth_it);
//  gb.comulativeBoundary(boundarys);
//  //gb.useFlow(flow);
//  SubRegion region = gb.segmentCorrectImage(c_curr, d_curr, c_prev, d_prev, label);
//
//  // �V�[���t���[�̐���
//  //Mat flow;
//  SceneFlow::sceneFlowFarneback(c_curr, d_curr, c_next, d_next, flow, scale,
//    levels, ap_flow, iterations, ap_flow / pow((float)(1 / scale), (float)(levels - 1)), 1,1);
//
//  // �����ʂ̌v�Z�̑O�����i�����̈�̋��E�ߖT�̃t���[�������j
//  Mat boundary;
//  label.release();
//  int region_num = preprocess(label, boundary, region);
//
//  // �����ʂ̌v�Z
//  Features f_vec = calcFeatures(label, frame_num, region_num,
//    c_curr, d_curr, flow, boundary);
//
//  // ���E�̎擾
//  boundarys.push_back(boundary);
//
//  char name[80];
//  string buf;
//
//  // �`�揈��
//  // �e�t���[���̕����̈��`��
//  sprintf(name, "./out_subregion/subregion-0%004d.png", frame_num);
//  buf = name;
//  Mat draw = drawSubRegion(label);
//  imwrite(buf, draw);
//  imshow("subregion", draw);
//  waitKey(10);
//
//  // �V�[���t���[�̃t�@�C���o��
//  FileOperation::writeSceneFlowDat(flow, frame_num);
//
//  return f_vec;
//}

/**
 * @brief ����������
 * @param[in, out] color �摜���
 * @param[in, out] depth �[�x���
 */
void SubRegionExtraction::smoothing(Mat &color, Mat &depth)
{
  // �`�����l�����ɕ�����
  Mat color32f, depth32f;
  color.convertTo(color32f, CV_32FC3);
  depth.convertTo(depth32f, CV_32FC1);

  vector<Mat> bgr;
  split(color32f, bgr);

  Mat smooth_r = bgr[2].clone();
  Mat smooth_g = bgr[1].clone();
  Mat smooth_b = bgr[0].clone();
  Mat smooth_d = depth32f.clone();

  if (sigma != 0.0) {
    // ���������J��Ԃ�
    for (int i = 0; i < smooth_it; i++) {
      // �o�C���e�����t�B���^�𗘗p
      if (color_sigma != 0.0) {
        Smooth::bilateralFiler(smooth_r, smooth_r, ap_filter, color_sigma, sigma);
        Smooth::bilateralFiler(smooth_g, smooth_g, ap_filter, color_sigma, sigma);
        Smooth::bilateralFiler(smooth_b, smooth_b, ap_filter, color_sigma, sigma);
        Smooth::bilateralFiler(smooth_d, smooth_d, ap_filter, color_sigma, sigma, 0);
      }
      // �K�E�V�A���t�B���^�𗘗p
      else {
        Smooth::gaussianFilter(smooth_r, smooth_r, ap_filter, sigma);
        Smooth::gaussianFilter(smooth_g, smooth_g, ap_filter, sigma);
        Smooth::gaussianFilter(smooth_b, smooth_b, ap_filter, sigma);
        Smooth::gaussianFilter(smooth_d, smooth_d, ap_filter, sigma, 0);
      }
    }
  }

  vector<Mat> mvc;
  mvc.push_back(smooth_b);
  mvc.push_back(smooth_g);
  mvc.push_back(smooth_r);
  
  merge(mvc, color);
  depth = smooth_d;
}

/**
 * @brief �O�����i���x���̎擾�ƕ����̈�̋��E�𒊏o�j
 * @param[out] label   �����̈�̃��x��
 * @param[out] boundary  �����̈�̋��E
 * @param[in] region  �����̈�
 * @return  �����̈�̐�
 */
int SubRegionExtraction::preprocess(Mat &label, Mat &boundary, SubRegion &region)
{
  int width = region.width;
  int height = region.height;

  label = Mat::ones(height, width, CV_32S) * -1;
  boundary = Mat::zeros(height, width, CV_8UC1);

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

      for (int yy = y; yy < y + 2 && yy < height; yy++) {
        for (int xx = x; xx < x + 2 && xx < width; xx++) {
          int comp2 = region.find(yy * width + xx);
          if (comp != comp2) {
            // �����̈�̋��E�̕`��
            line(boundary, Point(x, y), Point(x, y), 255);
          }
        }
      }
    }
  }

  return region_num;
}


/**
* @brief �����̈�̓����ʂ̌v�Z
* @param[out] label  ���x��
* @param[in]  frame_num �t���[���ԍ�
* @param[in]  region_num �����̈搔
* @param[in]  color   �摜���
* @param[in]  depth   �[�x���
* @param[in]  flow    �^�����i�V�[���t���[�j
* @param[in]  boundary  �����̈�̋��E
* @return    �����̈�̓����ʂ̃��X�g
*/
Features SubRegionExtraction::calcFeatures(Mat &label, const int frame_num,
  const int region_num, const Mat &color, const Mat &depth, const Mat &flow,
  const Mat &boundary)
{
  int width = color.cols;
  int height = color.rows;

  Features f(region_num);

  // �V�[���t���[�̃q�X�g�O�����̏�����
  vector<Mat> delxyz;
  split(flow, delxyz);

  Point3d min, max;
  minMaxLoc(delxyz[0], &min.x, &max.x);
  minMaxLoc(delxyz[1], &min.y, &max.y);
  minMaxLoc(delxyz[2], &min.z, &max.z);

  vector<vector<int>> histgram_x(region_num);
  vector<vector<int>> histgram_y(region_num);
  vector<vector<int>> histgram_z(region_num);
  for (int i = 0; i < region_num; i++) {
    histgram_x[i].resize(cvCeil(max.x) - cvFloor(min.x) + 1);
    histgram_y[i].resize(cvCeil(max.y) - cvFloor(min.y) + 1);
    histgram_z[i].resize(cvCeil(max.z) - cvFloor(min.z) + 1);
  }

  /////////////////////////////////////////////////////////////////////////////
  vector<vector<int>> median_x(region_num);
  vector<vector<int>> median_y(region_num);
  vector<vector<int>> median_z(region_num);
  /////////////////////////////////////////////////////////////////////////////

  // �����̈悲�Ƃɓ����ʂ̑������v�Z
  vector<int> flow_size(region_num);
  vector<int> remove_size(region_num);
  bool boundary_flag;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int l = label.at<int>(Point(x, y));

      // RGB-D�l�̑������v�Z
      Vec3f bgr = color.at<Vec3f>(Point(x, y));
      f[l].r += bgr[2];
      f[l].g += bgr[1];
      f[l].b += bgr[0];
      float d = depth.at<float>(Point(x, y));
      f[l].d += d;

      // �[�x�̌����l�̑������v�Z
      if (d == 0) {
        remove_size[l]++;
      }

      // �̈�T�C�Y�C�d�S���v�Z
      f[l].size++;
      f[l].centroid.x += x;
      f[l].centroid.y += y;

      // ���E�ߖT�̃t���[������
      boundary_flag = false;
      int rad = ap_flow / 2;
      for (int y2 = y - rad; y2 <= y + rad && y2 < height && 0 <= y2; y2++) {
        for (int x2 = x - rad; x2 <= x + rad && x2 < width && 0 <= x2; x2++) {
          if (boundary.at<uchar>(Point(x2, y2)) != 0) {
            boundary_flag = true;
          }
        }
      }

      if (boundary_flag) {
        continue;
      }

      // �V�[���t���[�̃q�X�g�O�����̍쐬
      histgram_x[l][cvRound(delxyz[0].at<float>(Point(x, y))) - cvFloor(min.x)]++;
      histgram_y[l][cvRound(delxyz[1].at<float>(Point(x, y))) - cvFloor(min.y)]++;
      histgram_z[l][cvRound(delxyz[2].at<float>(Point(x, y))) - cvFloor(min.z)]++;
      flow_size[l]++;

      // �V�[���t���[�̍ŕp�l���v�Z���邽�߂̃��X�g���쐬
      median_x[l].push_back(delxyz[0].at<float>(Point(x, y)));
      median_y[l].push_back(delxyz[1].at<float>(Point(x, y)));
      median_z[l].push_back(delxyz[2].at<float>(Point(x, y)));
    }
  }

  // �����̈悲�Ƃɓ����ʂ̕��ς��v�Z
  for (int i = 0; i < region_num; i++) {
    if (f[i].size != 0) {
      f[i].num = frame_num;
      f[i].label = i;
      f[i].r /= f[i].size;
      f[i].g /= f[i].size;
      f[i].b /= f[i].size;
      if (f[i].size == remove_size[i]) {
        f[i].d = 0;
      }
      else {
        f[i].d /= (f[i].size - remove_size[i]);
      }
      f[i].centroid.x /= f[i].size;
      f[i].centroid.y /= f[i].size;

      // �V�[���t���[�̃��[�h�̌v�Z
      if (flow_size[i] > 0) {
        f[i].flow2d.x = histgramMode(histgram_x[i], cvCeil(max.x), cvFloor(min.x), 1);
        f[i].flow2d.y = histgramMode(histgram_y[i], cvCeil(max.y), cvFloor(min.y), 1);
        f[i].flow2d.z = histgramMode(histgram_z[i], cvCeil(max.z), cvFloor(min.z), 1);

        ///////////////////////////////////////////////////////////////////////
        // �V�[���t���[�̒����l���v�Z
        // �e���X�g���\�[�g
        sort(median_x[i].begin(), median_x[i].end());
        sort(median_y[i].begin(), median_y[i].end());
        sort(median_z[i].begin(), median_z[i].end());

        if ((flow_size[i] % 2) != 0) {
          f[i].flow2d.x = median_x[i][cvFloor(flow_size[i] / 2.0)];
          f[i].flow2d.y = median_y[i][cvFloor(flow_size[i] / 2.0)];
          f[i].flow2d.z = median_z[i][cvFloor(flow_size[i] / 2.0)];
        }
        else {
          f[i].flow2d.x = (median_x[i][(flow_size[i] / 2)] + 
            median_x[i][(flow_size[i] / 2) - 1]) / 2;
          f[i].flow2d.y = (median_y[i][(flow_size[i] / 2)] + 
            median_y[i][(flow_size[i] / 2) - 1]) / 2;
          f[i].flow2d.z = (median_z[i][(flow_size[i] / 2)] + 
            median_z[i][(flow_size[i] / 2) - 1]) / 2;
        }

        ///////////////////////////////////////////////////////////////////////

        // �V�[���t���[�̊O��l������
        if (abs(f[i].flow2d.z) > FLOW_THRESHOLD) {
          f[i].flow2d.z = 0;
        }
      }
      else {
        f[i].flow2d.x = 0;
        f[i].flow2d.y = 0;
        f[i].flow2d.z = 0;
      }

      // �m�����̌v�Z
      f[i].norm2d = sqrt(square(f[i].flow2d.x) + square(f[i].flow2d.y)
        + square(f[i].flow2d.z));

      // �V�[���t���[��mm�ɕϊ��F�菇�P�C���z�I�Ȏ��̃t���[���̏d�S�ʒu���v�Z
      f[i].flow3d = depth2World(f[i].centroid.x + f[i].flow2d.x,
        f[i].centroid.y + f[i].flow2d.y, luminance2Depth(f[i].d + f[i].flow2d.z),
        width, height);

      // �d�S�ʒu��mm�ɕϊ�
      if (f[i].d != 0.0) {
        f[i].centroid3d = 
          depth2World(f[i].centroid.x, f[i].centroid.y, luminance2Depth(f[i].d), width, height);
      }
      else {
        f[i].centroid3d = 
          depth2World(f[i].centroid.x, f[i].centroid.y, 0, width, height);
      }

      // �V�[���t���[��mm�ɕϊ��F�菇�Q
      f[i].flow3d = f[i].flow3d - f[i].centroid3d;

      // �m�����̌v�Z
      f[i].norm3d = sqrt(square(f[i].flow3d.x) + square(f[i].flow3d.y)
        + square(f[i].flow3d.z));
    }
  }

  return f;
}

/**
* @brief ���[�h�i�ŕp�l�j�̌v�Z
* @param[in] histgram  �q�X�g�O����
* @param[in] max �v�f�̍ő�l
* @param[in] min �v�f�̍ŏ��l
* @param[in] interval  �ʎq���Ԋu
* @return  �q�X�g�O�����̃��[�h
*/
float SubRegionExtraction::histgramMode(const vector<int> &histgram,
  const int max, const int min, const float interval)
{
  int max_elem = 0, argmax = 0;
  int range = (max - min) / interval;

  for (int i = 0; i <= range; i++) {
    if (max_elem < histgram.at(i)) {
      max_elem = histgram.at(i);
      argmax = i;
    }
  }

  return argmax * interval + min;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief �I�v�e�B�J���t���[��dat�t�@�C���ɏo��
 * @param[in] flow  �I�v�e�B�J���t���[
 * @param[in] num   �t���[���ԍ�
 * @param[in] interval  �t���[�̏o�͊Ԋu
 */
void outOpticalFlowDat(const Mat &flow, //const Mat &boundary, const int aperture,
  const int num, const int interval)
{
  int width = flow.cols;
  int height = flow.rows;

  // ���͉摜�̋P�x�l���t�@�C���ɏ�������
  char name[80];
  sprintf(name, "./file/opticalflow%d.dat", num);
  FILE *fp = fopen(name, "w");

  //bool boundary_flag;
  //int rad = aperture / 2;
  for (int y = 0; y < height; y+= interval) {
    for (int x = 0; x < width; x+= interval) {
      Point3f velocity = flow.at<Point3f>(Point(x, y));

      // �G�b�W�ߖT�̃t���[������
      //boundary_flag = false;
      //for (int y2 = y - rad; y2 <= y + rad && y2 < height && 0 <= y2; y2++) {
      //  for (int x2 = x - rad; x2 <= x + rad && x2 < width && 0 <= x2; x2++) {
      //    if (boundary.at<uchar>(Point(x2, y2)) != 0) {
      //      boundary_flag = true;
      //    }
      //  }
      //}

//      if (!boundary_flag) {
      float norm = sqrtf(square(velocity.x) + square(velocity.y) + square(velocity.z));
      if (abs(velocity.z) > 2 && abs(velocity.z) < 10)
        fprintf(fp, "%d, %d, %f, %f\n", x, y, velocity.x, velocity.y);
      //}
      //else {
      //  fprintf(fp, "%d, %d, %f, %f\n", x, y, 0, 0);
      //}
    }
//    fprintf(fp, "\n");
  }

  fclose(fp);
}

/**
 * @brief �V�[���t���[��dat�t�@�C���ɏo��
 * @param[in] depth  �[�x���
 * @param[in] flow  �V�[���t���[
 * @param[in] num   �t���[���ԍ�
 * @param[in] interval  �t���[�̏o�͊Ԋu
 */
void outSceneFlowDat(const Mat &depth, const Mat &flow, const int num, const int interval)
{
  int width = depth.cols;
  int height = depth.rows;

  // ���͉摜�̋P�x�l���t�@�C���ɏ�������
  char buf[80];
  sprintf(buf, "./file/sceneflow%d.dat", num);
  FILE *fp = fopen(buf, "w");

  for (int y = 0; y < height; y+= interval) {
    for (int x = 0; x < width; x+= interval) {
      Point3f velocity = flow.at<Point3f>(Point(x, y));
      uchar z = depth.at<uchar>(Point(x, y));
      float norm = sqrtf(square(velocity.x) + square(velocity.y) + square(velocity.z));
      if (abs(velocity.z) > 2 && abs(velocity.z) < 10)
        fprintf(fp, "%d %d %d %f %f %f\n", x, z, y, velocity.x, velocity.z, velocity.y);
    }
  }

  fclose(fp);
}
