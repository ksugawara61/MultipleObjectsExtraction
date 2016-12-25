/*****************************************************************************/
/*! @addtogroup 
 *  @file   disjoint-set.cpp
 *  @brief
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "disjoint-set.h"

/**
 * @brief コンストラクタ
 * @param[in] width フレーム横幅
 * @param[in] height フレーム縦幅
 */
SubRegion::SubRegion(int width, int height)
{
  elts.resize(width * height);
  num = width * height;
  for (int i = 0; i < num; i++) {
    elts[i].rank = 0;
    elts[i].size = 1;
    elts[i].p = i;
  }

  this->width = width;
  this->height = height;
}

/**
 * @brief デストラクタ
 */
SubRegion::~SubRegion(void)
{
}

/**
 * @brief 要素のラベルの探索
 * @param[in] x 要素番号
 * @return  ラベル
 */
int SubRegion::find(int x) {
  int y = x;
  while (y != elts[y].p)
    y = elts[y].p;
  elts[x].p = y;
  return y;
}

/**
 * @brief 要素の追加（ノードの統合）
 * @param[in] x
 * @param[in] y
 */
void SubRegion::join(int x, int y) {
  if (elts[x].rank > elts[y].rank) {
    elts[y].p = x;
    elts[x].size += elts[y].size;
  } else {
    elts[x].p = y;
    elts[y].size += elts[x].size;
    if (elts[x].rank == elts[y].rank)
      elts[y].rank++;
  }
  num--;
}

/**
 * @brief 領域サイズを返す
 * @param[in] x 要素番号
 * @return  領域サイズ
 */
int SubRegion::size(int x) const { return elts[x].size; }

/**
 * @brief 要素数を返す
 * @return 要素（領域）数
 */
int SubRegion::num_sets() const { return num; }


void SubRegion::join_root(int x, int y)
{
  elts[y].p = x;
  elts[x].size += elts[y].size;

  if (elts[x].rank == elts[y].rank) {
    elts[y].rank++;
  }

  num--;
}
