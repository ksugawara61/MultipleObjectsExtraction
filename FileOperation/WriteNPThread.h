/*****************************************************************************/
/*! @addtogroup 
 *  @file   WriteNPThread.h
 *  @brief  最近点対の計算・ファイル書き込みに関するファイル（ヘッダファイル）
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
 *  @brief  最近点対の計算・ファイル書き込みを行うスレッドクラス
 */
class WriteNearestThread
{
public:
  WriteNearestThread(Features f, int seq_num, Mat label, int t, int start, int end);
  ~WriteNearestThread(void);
  void createThread(int N);

private:
  HANDLE	h_thread;	        /*!< スレッドのハンドラ */
	bool	flag;		            /*!< スレッド制御フラグ */
  
  Features f;   /*!< 対象フレームでの領域系列の集合に属する部分領域 */
  int seq_num;  /*!< 領域系列数 */
  Mat label;    /*!< 部分領域のラベル */
  Mat depth;    /*!< 深度情報 */
  Mat boundary; /*!< 部分領域の境界 */
  int t;        /*!< フレーム番号 */
  int start;    /*!< 先頭フレーム番号 */
  int end;      /*!< 末尾フレーム番号 */
  int N;        /*!< 最近点対の数 */

  static unsigned __stdcall run(void *param);
  void writeNearestPointDat(void);
  void writeNearestPointDatUsingBoundary(void);
};

#endif