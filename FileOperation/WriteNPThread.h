/*****************************************************************************/
/*! @addtogroup 
 *  @file   WriteNPThread.h
 *  @brief  �ŋߓ_�΂̌v�Z�E�t�@�C���������݂Ɋւ���t�@�C���i�w�b�_�t�@�C���j
 *  @date   
 *  @author ksugawara
******************************************************************************/
#ifndef WRITEDISTANCETHREAD_H
#define WRITEDISTANCETHREAD_H

#include <Windows.h>
#include <process.h>
#include "../Miscellaneous/Misc.h"

#ifdef _DEBUG
#pragma comment(lib, "../x64/Debug/Miscellaneousd.lib")
#else
#pragma comment(lib, "../x64/Release/Miscellaneous.lib")
#endif

/*! @class  WriteNearestThread
 *  @brief  �ŋߓ_�΂̌v�Z�E�t�@�C���������݂��s���X���b�h�N���X
 */
class WriteNearestThread
{
public:
  WriteNearestThread(Features f, int seq_num, Mat label, int t, int start, int end);
  ~WriteNearestThread(void);
  void createThread(int N);

private:
  HANDLE	h_thread;	        /*!< �X���b�h�̃n���h�� */
	bool	flag;		            /*!< �X���b�h����t���O */
  
  Features f;   /*!< �Ώۃt���[���ł̗̈�n��̏W���ɑ����镔���̈� */
  int seq_num;  /*!< �̈�n�� */
  Mat label;    /*!< �����̈�̃��x�� */
  Mat depth;    /*!< �[�x��� */
  Mat boundary; /*!< �����̈�̋��E */
  int t;        /*!< �t���[���ԍ� */
  int start;    /*!< �擪�t���[���ԍ� */
  int end;      /*!< �����t���[���ԍ� */
  int N;        /*!< �ŋߓ_�΂̐� */

  static unsigned __stdcall run(void *param);
  void writeNearestPointDat(void);
  void writeNearestPointDatUsingBoundary(void);
};

#endif