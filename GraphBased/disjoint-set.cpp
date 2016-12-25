/*****************************************************************************/
/*! @addtogroup 
 *  @file   disjoint-set.cpp
 *  @brief
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "disjoint-set.h"

/**
 * @brief �R���X�g���N�^
 * @param[in] width �t���[������
 * @param[in] height �t���[���c��
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
 * @brief �f�X�g���N�^
 */
SubRegion::~SubRegion(void)
{
}

/**
 * @brief �v�f�̃��x���̒T��
 * @param[in] x �v�f�ԍ�
 * @return  ���x��
 */
int SubRegion::find(int x) {
  int y = x;
  while (y != elts[y].p)
    y = elts[y].p;
  elts[x].p = y;
  return y;
}

/**
 * @brief �v�f�̒ǉ��i�m�[�h�̓����j
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
 * @brief �̈�T�C�Y��Ԃ�
 * @param[in] x �v�f�ԍ�
 * @return  �̈�T�C�Y
 */
int SubRegion::size(int x) const { return elts[x].size; }

/**
 * @brief �v�f����Ԃ�
 * @return �v�f�i�̈�j��
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
