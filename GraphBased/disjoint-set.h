/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/*****************************************************************************/
/*! @addtogroup 
 *  @file   disjoint-set.h
 *  @brief  �i�w�b�_�t�@�C���j
 *  @date   
 *  @author ksugawara
******************************************************************************/

#ifndef DISJOINT_SET
#define DISJOINT_SET

#include <vector>

using namespace std;

// disjoint-set forests using union-by-rank and path compression (sort of).

/*! @struct uni_elt
 *  @brief  �����̈�ւ̕����v�Z�p�̍\����
 */
struct uni_elt {
  int rank;   /*!< �����̈�̃����N */
  int p;      /*!< �����̈�̃��x���̃|�C���^�H */
  int size;   /*!< ��f�� */
};

/*! @class  SubRegion
 *  @brief  �����̈�̃N���X�\����
 */
class SubRegion {
public:
  SubRegion(int width, int height);
  ~SubRegion(void);
  int find(int x);
  void join(int x, int y);
  int size(int x) const;
  int num_sets(void) const;
  void join_root(int x, int y);

  int width;
  int height;

private:
  vector<uni_elt> elts;
  int num;
};

#endif
