/*****************************************************************************/
/*! @addtogroup 
 *  @file   MotionVector.h
 *  @brief  �^�����̏����Ɋւ��郉�C�u�����i�w�b�_�t�@�C���j
 *  @date   
 *  @author ksugawara
******************************************************************************/
#ifndef MOTIONVECTOR_H
#define MOTIONVECTOR_H

/* �w�b�_�t�@�C�� */
#include <opencv2\opencv.hpp>
#include <opencv2\legacy\legacy.hpp>		// BruteForceMatche�ɕK�v
#include <opencv2\superres\optical_flow.hpp>
#include <opencv2\ocl\ocl.hpp>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// �o�[�W�������̎擾
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

// lib�t�@�C�����̍Ō�̕�����Release��Debug�ŕ�����
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif

/* �����J�̐ݒ� */
#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_objdetect" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_contrib" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_features2d" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_flann" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_gpu" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_legacy" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_ts" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_video" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_nonfree" CV_VERSION_STR CV_EXT_STR)	// SURF
#pragma comment(lib,"opencv_superres" CV_VERSION_STR CV_EXT_STR) //DualTVL1

#endif

/* ���O��� */
using namespace cv;
using namespace cv::superres;

/*! @namespace OpticalFlow
 *  @brief  �I�v�e�B�J���t���[�v�Z�p�̖��O���
 */
namespace OpticalFlow
{
  void opticalFlowBM(Mat prev, Mat next, Mat &flow, 
    Size block = Size(10, 10), Size shift = Size(1, 1),
    Size max_range = Size(50, 50), int use_previous = 0);
  void opticalFlowHS(Mat prev, Mat next, Mat &flow,
    int use_previous = 0, double lambda = 100.0,
    CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 64, 0.01));
  void opticalFlowLK(Mat prev, Mat next, Mat &flow,
    Size win_size = Size(15, 15));
  void opticalFlowPyrLK(Mat prev, Mat next, Mat &flow, 
    Size win_size = Size(15, 15), int levels = 3, 
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
    double deriv_lambda = 0.5, int flags = 0);
  void opticalFlowFarneback(Mat prev, Mat next, Mat &flow,
    double pyrscale = 0.5, int levels = 3, int winsize = 15, int iterations = 3, 
    int polyN = 5, double polysigma = 1.1, int flags = 0);

  void opticalFlowDualTVL1(Mat prev, Mat next, Mat &flow);
  void opticalFlowSimple(Mat prev, Mat next, Mat &flow);
}

/*! @namespace SceneFlow
 *  @brief  �V�[���t���[�v�Z�p�̖��O���
 */
namespace SceneFlow
{
  void sceneFlowBM(Mat c_prev, Mat d_prev,  Mat c_next, Mat d_next, Mat &flow, 
    Size block = Size(10, 10), Size shift = Size(1, 1),
    Size max_range = Size(50, 50), int use_previous = 0);
  void sceneFlowHS(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow,
    int use_previous = 0, double lambda = 100.0,
    CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 64, 0.01));
  void sceneFlowLK(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow,
    Size win_size = Size(15, 15));
  void sceneFlowPyrLK(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow, 
    Size win_size = Size(15, 15), int levels = 3, 
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
    double deriv_lambda = 0.5, int flags = 0);
  void sceneFlowFarneback(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow,
    double pyrscale = 0.5, int levels = 3, int winsize = 15, int iterations = 3, 
    int polyN = 5, double polysigma = 1.1, int flags = 0);
  void sceneFlowDualTVL1(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow);
  void sceneFlowSimple(Mat c_prev, Mat d_prev, Mat c_next, Mat d_next, Mat &flow);

  void calcVelZ(const Mat &d_prev, const Mat &d_next, Mat &flow);
}

#endif