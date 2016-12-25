/*****************************************************************************/
/*! @addtogroup 
 *  @file   SubRegionMatching.cpp
 *  @brief  領域系列の決定処理に関するファイル
 *  @date   
 *  @author ksugawara
******************************************************************************/

#include "SubRegionMatching.h"

// 部分領域の対応付け
/**
 * @brief コンストラクタ
 * @param[in] start       先頭フレーム番号
 * @param[in] end         末尾フレーム番号
 * @param[in] norm        移動量の閾値
 * @param[in] w_threshold エッジの重みの閾値
 * @param[in] w_region    領域サイズ・重心の重み係数
 * @param[in] w_rgbd      画像・深度・運動情報の重み係数
 * @param[in] w_I         画像情報の重み係数
 * @param[in] w_D         深度情報の重み係数
 * @param[in] w_F         運動情報（シーンフロー）の重み係数
 */
SubRegionMatching::SubRegionMatching(const int start, const int end,
  const float norm, const float w_threshold, const float w_region,
  const float w_rgbd, const float w_I, const float w_D, const float w_F)
{
  this->start = start;
  this->end = end;
  this->norm_threshold = norm;
  this->w_threshold = w_threshold;

  this->w_region = w_region;  // 0.2
  this->w_rgbd = w_rgbd;      // 1.5
  this->w_I = w_I;
  this->w_D = w_D;            // 3.0
  this->w_F = w_F;
}

/**
 * @brief デストラクタ
 */
SubRegionMatching::~SubRegionMatching(void)
{
}

//TemporalSubRegion SubRegionMatching::subRegionMatching(vector<Features> &f_vec)
//{
//  //// ラベルの確保
//  //vector<Mat> labels = readMultipleLabelDat(start, end, 640, 480);
//  //// グラフの構築
//  //vector<TemporalEdge> edges;
//  //int size = f_vec.size();
//  //for (int i = 0; i < size; i++) {
//  //  for (int j = 0; j < f_vec.at(i).size(); j++) {
//  //    Feature &p = f_vec.at(i).at(j);
//  //    if (p.norm2d > norm) {
//  //      // 前フレーム
//  //      overlapSubRegionConnection(edges, p, f_vec, false, labels);
//  //      // 次フレーム
//  //      overlapSubRegionConnection(edges, p, f_vec, true, labels);
//  //    }
//  //  }
//  //}
//
//  // グラフの構築
//  vector<TemporalEdge> edges;
//  int size = f_vec.size();
//  for (int i = 0; i < size; i++) {
//    for (int j = 0; j < f_vec.at(i).size(); j++) {
//      Feature &p = f_vec.at(i).at(j);
//      if (p.norm3d > norm_threshold) {
//        // 前フレーム
//        edgeConnection(edges, p, f_vec, false);
//        // 次フレーム
//        edgeConnection(edges, p, f_vec, true);
//      }
//    }
//  }
//
//  // 部分領域の対応付け
//  int num_edges = edges.size();
//  cout << num_edges << endl;
//
//  vector<int> num_vertices(f_vec.size());
//  for (int i = 0; i < f_vec.size(); i++) {
//    num_vertices.at(i) = f_vec.at(i).size();
//  }
//
//  TemporalSubRegion region(f_vec.size(), num_vertices, f_vec);
//
//  // 重み順にエッジをソート
//  sort(edges.begin(), edges.end());
//
//  for (int i = 0; i < num_edges; i++) {
//    TemporalEdge &pedge = edges.at(i);
//
//    // 部分領域の対応付け
//    int f_prev = 0, l_prev = 0, r_prev = 0, f_next = 0, l_next = 0, r_next = 0, buf;
//    region.find(pedge.p.frame, pedge.p.label, f_prev, l_prev, r_prev, buf);
//    region.find(pedge.n.frame, pedge.n.label, f_next, l_next, r_next, buf);
//
//    if (r_next == 0 && pedge.w < w_threshold) {
//      // 対応付け
//      region.join(f_prev, l_prev, f_next, l_next);
//    }
//  }
//
//  return region;
//}

/**
 * @brief 領域系列を決定する処理
 * @param[in,out]   f_vec 各フレームでの部分領域の特徴量のリスト
 * @param[in] labels  部分領域のラベルのリスト
 * @param[in] s     先頭フレーム番号
 * @param[in] e     末尾フレーム番号
 * @param[in,out]  result 前の系列での最後のフレームの対応付け結果，
 *                        現在の系列での最後のフレームの対応付け結果を格納
 *                        （※ストリーミング処理を実装できていないため，利用不可）
 * @return 決定した領域系列のリスト
 */
RegionSequences SubRegionMatching::streamingSubRegionMatching(vector<Features> &f_vec, 
  vector<Mat> labels, const int s, const int e, Features &result)
{
  start = s;
  end = e;

  // 前の系列での最後のフレームの対応付け結果をリストに追加
  if (!result.empty()) {
    start = s - 1;
    f_vec.insert(f_vec.begin(), result);  // 先頭に要素を追加
  }

  //// 前の系列での最後のフレームの対応付け結果をリストに追加
  //if (result.empty()) {
  //  // 最初のフレームの場合何もしない
  //  start = s;
  //  end = e;
  //}
  //else {
  //  start = s - 1;
  //  end = e;
  //  f_vec.insert(f_vec.begin(), result);  // 先頭に要素を追加
  //}

  // グラフの構築
  vector<TemporalEdge> edges;
  int size = f_vec.size();
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < f_vec.at(i).size(); j++) {
      Feature &p = f_vec.at(i).at(j);
      if (p.norm3d > norm_threshold) {
        // 前フレーム
        overlapSubRegionConnection(edges, p, f_vec, false, labels);
        // 次フレーム
        overlapSubRegionConnection(edges, p, f_vec, true, labels);
      }
    }
  }

  //// グラフの構築
  //vector<TemporalEdge> edges;
  //int size = f_vec.size();
  //for (int i = 0; i < size; i++) {
  //  for (int j = 0; j < f_vec.at(i).size(); j++) {
  //    Feature &p = f_vec.at(i).at(j);
  //    if (p.norm3d > norm_threshold) {
  //      // 前フレーム
  //      edgeConnection(edges, p, f_vec, false);
  //      // 次フレーム
  //      edgeConnection(edges, p, f_vec, true);
  //    }
  //  }
  //}

  // 部分領域の対応付け
  int num_edges = edges.size();
  cout << "edge num : " << num_edges << endl;

  vector<int> num_vertices(f_vec.size());
  for (int i = 0; i < f_vec.size(); i++) {
    num_vertices.at(i) = f_vec.at(i).size();
  }

  TemporalRegion region(f_vec.size(), num_vertices);

  // 重み順にエッジをソート
  sort(edges.begin(), edges.end());

  // 部分領域を対応付け
  for (int i = 0; i < num_edges; i++) {
    TemporalEdge &pedge = edges.at(i);

    TemporalVertex a = region.find(pedge.p);
    TemporalVertex b = region.find(pedge.n);
    if (a != b) {
      if (pedge.w < w_threshold) {
        region.joinRoot(a, b);
        f_vec.at(pedge.p.frame).at(pedge.p.label).next.push_back(pedge);
        f_vec.at(pedge.n.frame).at(pedge.n.label).prev.push_back(pedge);
      }
    }
  }

  //// 最後のフレームの対応付け結果を格納
  //result = f_vec.at(end - start - 1);
  //size = result.size();
  //for (int i = 0; i < size; i++) {
  //  int f = 0, l = 0, r_next = 0, r_prev = 0;
  //  region.find(end - start - 1, i, f, l, r_next, r_prev);
  //  result.at(i).norm3d = region.getNorm(end - start - 1, i); // 対象の部分領域として抽出するため移動量を更新
  //}
  
  return region2Sequences(region, f_vec);
}

/**
 * @brief 短い領域系列，移動の少ない領域系列を除去する処理
 * @param[in] region_seq  領域系列のリスト
 * @param[in] min_length  最少の領域系列の長さ
 * @param[in] min_move    最少の領域系列の移動量（平均）
 * @return  除去処理後の領域系列のリスト
 */
RegionSequences SubRegionMatching::removeSequences(RegionSequences region_seq, const int min_length, 
  const float min_move)
{
  // 各領域系列の移動量の計算
  for (int i = 0; i < region_seq.size(); i++) {
    region_seq.at(i).calcNorm();
  }

  // 短い領域系列，移動の少ない領域系列の除去
  for (int i = 0; i < region_seq.size(); i++) {
    if (region_seq.at(i).getLength() <= min_length || 
      region_seq.at(i).getNorm() <= min_move) 
    {
      region_seq.erase(region_seq.begin() + i);
      i--;
    }
  }

  return region_seq;
}

/**
 * @brief エッジの接続（対応付け候補の選択）
 * @param[out]  edges   エッジのリスト
 * @param[in] f         対象の部分領域の特徴量
 * @param[in] f_vec     各フレームの部分領域の特徴量
 * @param[in] time      true : 次フレーム false : 前フレーム
 */
void SubRegionMatching::edgeConnection(vector<TemporalEdge> &edges, Feature &f,
  vector<Features> &f_vec, const bool time)
{
  int frame = f.num - start;

  // 前フレーム
  if (!time) {
    if (frame > 0) {
      TemporalEdge e;

      Features &prev = f_vec.at(frame - 1);
      for (int i = 0; i < prev.size(); i++) {
        e.p.frame = frame - 1;
        e.p.label = prev.at(i).label;
        e.n.frame = frame;
        e.n.label = f.label;
        e.pflag = true;

        // 既に存在するエッジか探索
        if (findTemporalEdge(edges, e)) {
          continue;
        }

        // 重心距離が一定距離内か判定
        /*Point2f flow(prev.at(i).flow2d.x, prev.at(i).flow2d.y);
        if (distanceCentroid(prev.at(i).centroid, f.centroid, flow) > dist) {
          continue;
        }*/

        // 重みの計算
        e.w = weightTemporalEdge(prev.at(i), f);

        if (e.w < w_threshold) {
//          check(prev.at(i), f);

          edges.push_back(e);

          // 再帰処理（前フレームの場合，次のフレームでも同様の処理）
          edgeConnection(edges, prev.at(i), f_vec, false);
//          edgeConnection(edges, prev.at(i), f_vec, true);
        }
      }
    }
  }
  // 次フレーム
  else {
    if (frame + 1 < f_vec.size()) {
      TemporalEdge e;

      Features &next = f_vec.at(frame + 1);
      for (int i = 0; i < next.size(); i++) {
        e.p.frame = frame;
        e.p.label = f.label;
        e.n.frame = frame + 1;
        e.n.label = next.at(i).label;
        e.nflag = true;

        // 既に存在するエッジか探索
        if (findTemporalEdge(edges, e)) {
          continue;
        }

        // 重心距離が一定距離内か判定
        //Point2f flow(f.flow2d.x, f.flow2d.y);
        //if (distanceCentroid(f.centroid, next.at(i).centroid, flow) > dist) {
        //  continue;
        //}

        // 重みの計算
        e.w = weightTemporalEdge(f, next.at(i));
        
        if (e.w < w_threshold) {
//          check(f, next.at(i));

          edges.push_back(e);

          // 再帰処理
//          edgeConnection(edges, next.at(i), f_vec, false);
          edgeConnection(edges, next.at(i), f_vec, true);
        }
      }
    }
  }
}

/**
 * @brief エッジの探索
 * @param[in] edges エッジのリスト
 * @param[in] e  探索するエッジ
 * @return  true エッジ有り　false エッジ無し
 */
bool SubRegionMatching::findTemporalEdge(vector<TemporalEdge> &edges, TemporalEdge &e)
{
  return (count(edges.begin(), edges.end(), e) != 0);
}

/**
 * @brief エッジの探索
 * @param[in] edges   エッジのリスト
 * @param[in] e       探索するエッジ
 * @param[in] time    true : 次フレーム  false : 前フレーム
 * @return  true 対応付け処理が不要　false 対応付け処理が必要
 */
bool SubRegionMatching::findTemporalEdge(vector<TemporalEdge> &edges, TemporalEdge &e, const bool time)
{
  vector<TemporalEdge>::iterator &pedge = find(edges.begin(), edges.end(), e);

  if (pedge != edges.end()) {
    if (time) {
      if (pedge->nflag) {
        return true;
      }
      else {
        pedge->nflag = true;
        return false;
      }
    }
    else {
      if (pedge->pflag) {
        return true;
      }
      else {
        pedge->pflag = true;
        return false;
      }
    }
  }
  else if (*pedge == e) {
    if (time) {
      if (pedge->nflag) {
        return true;
      }
      else {
        pedge->nflag = true;
        return false;
      }
    }
    else {
      if (pedge->pflag) {
        return true;
      }
      else {
        pedge->pflag = true;
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief エッジの重みの計算
 * @param[in] prev  前フレームの部分領域の特徴量
 * @param[in] next  次フレームの部分領域の特徴量
 * @return  エッジの重み
 */
float SubRegionMatching::weightTemporalEdge(Feature &prev, Feature &next)
{
  // 重心距離の計算
  Point2f flow(prev.flow2d.x, prev.flow2d.y);
  float dist_centroid = distanceCentroid(prev.centroid, next.centroid, flow);

  //// 領域サイズの比較（ユークリッド距離）
  //float dist_size = sqrtf(square(prev.size - next.size));

  // RGB-Dの相違度
  float dist_rgbd;
  if (abs(prev.flow2d.z) < FLOW_THRESHOLD) {
    dist_rgbd = sqrtf(square(prev.r - next.r)
      + square(prev.g - next.g)
      + square(prev.b - next.b)
      + (square((prev.d + prev.flow2d.z) - next.d) * w_D));
  }
  else {  // シーンフローの測定値が怪しいときは補正を行わないようにする
    dist_rgbd = sqrtf(square(prev.r - next.r)
      + square(prev.g - next.g)
      + square(prev.b - next.b)
      + (square(prev.d - next.d) * w_D));
  }
  
  // 領域サイズの比較
  float join = max(prev.size, next.size);
  float meet = min(prev.size, next.size);

  float dist_size = join * dist_centroid / meet;

  float compare = w_region * dist_size + w_rgbd * dist_rgbd;

  return compare;
}

/**
 * @brief フレーム間で重なり合う部分領域とエッジを接続
 * @param[out]  edges   エッジのリスト
 * @param[in]   f       対象の部分領域の特徴量
 * @param[in]   f_vec   各フレームでの部分領域の特徴量
 * @param[in]   time    true 次フレーム  false 前フレーム
 * @param[in]   labels  部分領域のラベルのリスト
 */
void SubRegionMatching::overlapSubRegionConnection(vector<TemporalEdge> &edges, Feature &f,
  vector<Features> &f_vec, const bool time, vector<Mat> &labels)
{
  int frame = f.num - start;
  int width;
  int height;

  // 前フレーム
  if (!time) {
    if (frame > 0) {
      width = labels.at(frame).cols;
      height = labels.at(frame).rows;

      Features &prev = f_vec.at(frame - 1);

      // 重なり合う部分領域の探索と重なっている画素数を計算
      vector<int> overlap(prev.size());
      Mat &current = labels.at(frame);
      Mat &previous = labels.at(frame - 1);
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          int l = previous.at<int>(y, x);

          int tmpx = min((int)(x + prev.at(l).flow2d.x), width - 1);
          tmpx = max(0, tmpx);
          int tmpy = min((int)(y + prev.at(l).flow2d.y), height - 1);
          tmpy = max(0, tmpy);

          if (current.at<int>(tmpy, tmpx) == f.label) {
            overlap.at(l)++;
          }
        }
      }

      for (int i = 0; i < prev.size(); i++) {
        if (overlap.at(i) == 0) {
          continue;
        }

        TemporalEdge e;
        e.p = TemporalVertex(frame - 1, prev.at(i).label);
        e.n = TemporalVertex(frame, f.label);
        e.pflag = true;

        // 既に存在するエッジか探索
        if (findTemporalEdge(edges, e)) {

          //// エッジを追加せずに再帰処理
          //if (!findTemporalEdge(edges, e, time)) {
          //  overlapSubRegionConnection(edges, prev.at(i), f_vec, false, labels);
          //}

          continue;
        }

        // 重みの計算
        e.w = weightOverlagEdge(prev.at(i), f, overlap.at(i));

        if (e.w < w_threshold) {
          edges.push_back(e);

          // 再帰処理
          overlapSubRegionConnection(edges, prev.at(i), f_vec, false, labels);
        }
      }
    }
  }
  // 次フレーム
  else {
    if (frame + 1 < f_vec.size()) {
      width = labels.at(frame).cols;
      height = labels.at(frame).rows;

      Features &next = f_vec.at(frame + 1);

      // 重なり合う部分領域の探索と重なっている画素数を計算
      vector<int> overlap(next.size());
      Mat &current = labels.at(frame);
      Mat &nexto = labels.at(frame + 1);
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          if (current.at<int>(y, x) == f.label) {
            int tmpx = min((int)(x + f.flow2d.x), width - 1);
            tmpx = max(0, tmpx);
            int tmpy = min((int)(y + f.flow2d.y), height - 1);
            tmpy = max(0, tmpy);

            int l = nexto.at<int>(tmpy, tmpx);
            overlap.at(l)++;
          }
        }
      }

      for (int i = 0; i < next.size(); i++) {
        if (overlap.at(i) == 0) {
          continue;
        }

        TemporalEdge e;
        e.p = TemporalVertex(frame, f.label);
        e.n = TemporalVertex(frame + 1, next.at(i).label);
        e.nflag = true;

        // 既に存在するエッジか探索
        if (findTemporalEdge(edges, e)) {

          //// エッジを追加せずに再帰処理
          //if (!findTemporalEdge(edges, e, time)) {
          //  overlapSubRegionConnection(edges, next.at(i), f_vec, true, labels);
          //}

          continue;
        }

        // 重みの計算
        e.w = weightOverlagEdge(f, next.at(i), overlap.at(i));

        if (e.w < w_threshold) {
          edges.push_back(e);

          // 再帰処理
          overlapSubRegionConnection(edges, next.at(i), f_vec, true, labels);
        }
      }
    }
  }
}

/**
 * @brief フレーム間での重なりを相違度の計算に利用
 * @param[in] prev    前フレームでの部分領域の特徴量
 * @param[in] next    次フレームでの部分領域の特徴量
 * @param[in] overlap 部分領域同士が重なり合う画素の数
 * @return  フレーム間での部分領域同士の相違度
 */
float SubRegionMatching::weightOverlagEdge(Feature &prev, Feature &next, int overlap)
{
  // RGB-Dの相違度
  float diff_i = w_I * sqrtf(square(prev.r - next.r) + square(prev.g - next.g)
      + square(prev.b - next.b));
  float diff_d = w_D * sqrtf(square((prev.d + prev.flow2d.z) - next.d));
  ////////////////////
  // 補正なし
  //float diff_d = w_D * sqrtf(square(prev.d - next.d));
  /////////////
  float diff_f = w_F * sqrtf(square(prev.flow3d.x - next.flow3d.x)
    + square(prev.flow3d.y - next.flow3d.y)
    + square(prev.flow3d.z - next.flow3d.z));
  float diff_h = diff_i + diff_d + diff_f;
  
  // 領域サイズ・重心の相違度
  Point2f flow(prev.flow2d.x, prev.flow2d.y);
  float dist_centroid = distanceCentroid(prev.centroid, next.centroid, flow);

  float diff_g = (prev.size + next.size) * dist_centroid / overlap;

  float diff = w_region * diff_g + w_rgbd * diff_h;

  return diff;
}

/**
 * @brief 対応付け結果を領域系列に変換
 * @param[in] region  部分領域の対応付け結果
 * @param[in] f_vec   各フレームでの部分領域の特徴量
 * @param[in] index   領域系列のインデックス
 * @return  領域系列のリスト
 */
RegionSequences SubRegionMatching::region2Sequences(TemporalRegion &region,
  vector<Features> &f_vec, vector<TemporalEdge> index)
{
  // 領域系列の決定
  RegionSequences region_seq;
  int length = f_vec.size();
  int region_num;

  for (int i = 0; i < length; i++) {
    region_num = f_vec.at(i).size();
    for (int j = 0; j < region_num; j++) {
      TemporalVertex label = region.find(TemporalVertex(i, j));
      
      // インデックスのリストが存在する場合，ルートのラベルを修正
      if (!index.empty()) {
        for (int k = 0; k < index.size(); k++) {
          if (index.at(k).p == label) {
            label = index.at(k).n;
          }
        }
      }

      int rank = region.findRank(TemporalVertex(i, j));

      // 対応付けされているかを判定
      if (rank != 0) {          
        bool flag = false;
        f_vec.at(i).at(j).root = label;

        // 領域系列へ追加
        int size = region_seq.size();
        for (int k = 0; k < size; k++) {
          if (region_seq.at(k).getLabel() == label) {
            region_seq.at(k).setFeature(i, f_vec.at(i).at(j));

            flag = true;
            break;
          }
        }

        // 新しい領域系列を作成
        if (!flag) {
          RegionSequence tmp(label, length);
          tmp.setFeature(i, f_vec.at(i).at(j));
          region_seq.push_back(tmp);
        }
      }
    }
  }

  return region_seq;
}

/**
 * @brief 領域系列の分割（※未完成）
 * @param[in] seq     領域系列
 * @param[in] labels  部分領域のラベルのリスト
 * @param[in] N       最近点対の数
 * @param[in] threshold 領域系列の分割の閾値
 * @return  分割した領域系列
 */
RegionSequences SubRegionMatching::sequenceSegmentation(RegionSequence seq, vector<Mat> labels,
  int N, float threshold)
{
  RegionSequences region_seq;
  vector<TemporalVertex> v(2);
  int length = labels.size();

  // 領域系列の分割が終了したか判定
  if (seq.segment) {
    // 領域系列内の相違度を評価
    float diff = diffSequence(v.at(0), v.at(1), seq, labels, N);
    cout << diff << endl;
    if (diff > threshold) {
      // エッジの重み順に対応付け

      // 分割のための対応付け処理の準備
      vector<Features> f_vec;
      for (int t = 0; t < length; t++) {
        f_vec.push_back(seq.getFeatures(t));
      }

      vector<int> num_vertices(f_vec.size());
      for (int t = 0; t < f_vec.size(); t++) {
        num_vertices.at(t) = f_vec.at(t).size();
      }

      TemporalRegion region(f_vec.size(), num_vertices);

      internalMatching(region, f_vec, v, v.at(0).frame, true);
      //    internalMatching(region, f_vec, v, v.at(0).frame, false);

      // 領域系列のルートのラベルを修正用のインデックス
      vector<TemporalEdge> index;
      TemporalEdge tmp_index;
      tmp_index.p = v.at(0);
      tmp_index.n = TemporalVertex(v.at(0).frame,
        f_vec.at(v.at(0).frame).at(v.at(0).label).label);
      index.push_back(tmp_index);
      tmp_index.p = v.at(1);
      tmp_index.n = TemporalVertex(v.at(1).frame,
        f_vec.at(v.at(1).frame).at(v.at(1).label).label);
      index.push_back(tmp_index);

      region_seq = region2Sequences(region, f_vec, index);
    }
    // 閾値を満たしている場合，分割を行わない
    else {
      seq.terminateSegment();
      region_seq.push_back(seq);
    }
  }
  else {
    region_seq.push_back(seq);
  }

  return region_seq;
}

/**
 * @brief 領域系列内部の対応付け（※未完成）
 * @param[in] region  領域系列内の部分領域
 * @param[in] f_vec   領域系列内の各フレームの部分領域の特徴量
 * @param[in] v       
 * @param[in] frame
 * @param[in] time    true 次フレーム  false 前フレーム
 */
void SubRegionMatching::internalMatching(TemporalRegion &region, const vector<Features> &f_vec,
  vector<TemporalVertex> v, int frame, const bool time)
{
  if (v.empty()) {
    return;
  }

  // 前のフレーム
  if (!time) {
    vector<TemporalVertex> prev_v;
    if (frame < 0) {
      return;
    }

    vector<TemporalEdge> edges;
    for (int i = 0; i < v.size(); i++) {
      edges.insert(edges.end(), f_vec.at(v.at(i).frame).at(v.at(i).label).prev.begin(), 
        f_vec.at(v.at(i).frame).at(v.at(i).label).prev.end());
    }

    // 重み順にエッジをソート
    sort(edges.begin(), edges.end());

    // 部分領域を対応付け
    for (int i = 0; i < edges.size(); i++) {
      TemporalEdge &pedge = edges.at(i);

      TemporalVertex a = TemporalVertex(-1, -1), b = TemporalVertex(-1, -1);
      int rank = -1;

      // 対応するインデックスを探索
      Features f = f_vec.at(pedge.p.frame);
      for (int j = 0; j < f.size(); j++) {
        if (f.at(j).label == pedge.p.label) {
          a = region.find(TemporalVertex(pedge.p.frame, j));
          rank = region.findRank(TemporalVertex(pedge.p.frame, j));
        }
      }      
      f = f_vec.at(pedge.n.frame);
      for (int j = 0; j < f.size(); j++) {
        if (f.at(j).label == pedge.n.label) {
          b = region.find(TemporalVertex(pedge.n.frame, j));
        }
      }

      if (a != b) {
        if (rank == 0) {
          region.joinRoot(b, a);
          prev_v.push_back(a);
        }
      }
    }
    
    // 再帰
    if (!prev_v.empty()) {
      internalMatching(region, f_vec, prev_v, frame - 1, false);
      internalMatching(region, f_vec, v, frame, true);
    }
  }
  // 次のフレーム
  else {
    vector<TemporalVertex> next_v;
    if (frame >= f_vec.size()) {
      return;
    }

    vector<TemporalEdge> edges;
    for (int i = 0; i < v.size(); i++) {
      edges.insert(edges.end(), f_vec.at(v.at(i).frame).at(v.at(i).label).next.begin(), 
        f_vec.at(v.at(i).frame).at(v.at(i).label).next.end());
    }

    // 重み順にエッジをソート
    sort(edges.begin(), edges.end());

    // 部分領域を対応付け
    for (int i = 0; i < edges.size(); i++) {
      TemporalEdge &pedge = edges.at(i);

      TemporalVertex a = TemporalVertex(-1, -1), b = TemporalVertex(-1, -1);
      int rank = -1;

      // 対応するインデックスを探索
      Features f = f_vec.at(pedge.p.frame);
      for (int j = 0; j < f.size(); j++) {
        if (f.at(j).label == pedge.p.label) {
          a = region.find(TemporalVertex(pedge.p.frame, j));
        }
      }      
      f = f_vec.at(pedge.n.frame);
      for (int j = 0; j < f.size(); j++) {
        if (f.at(j).label == pedge.n.label) {
          b = region.find(TemporalVertex(pedge.n.frame, j));
          rank = region.findRank(TemporalVertex(pedge.n.frame, j));
        }
      }

      if (a != b) {
        if (rank == 0) {
          region.joinRoot(a, b);
          next_v.push_back(b);
        }
      }
    }

    // 再帰
    if (!next_v.empty()) {
      internalMatching(region, f_vec, v, frame, false);
      internalMatching(region, f_vec, next_v, frame + 1, true);
    }
  }
}

/**
 * @brief 領域系列内の部分領域間の相違度を計算
 * @param[out]  v1    相違度が最大となる部分領域1
 * @param[out]  v2    相違度が最大となる部分領域2
 * @param[in] seq  対象の領域系列
 * @param[in] labels  各フレームでの部分領域のラベル
 * @param[in] N       最近点対の数
 * @return  領域系列内の相違度（部分領域間の相違度の最大値）
 */
float SubRegionMatching::diffSequence(TemporalVertex &v1, TemporalVertex &v2, 
  RegionSequence seq, vector<Mat> labels, int N)
{
  float max_diff = 0;   // 領域系列内の相違度
  int length = labels.size();

  for (int t = 0; t < length; t++) {
    Features f = seq.getFeatures(t);
    int subregion_num = f.size();

    vector<vector<float>> mat_diff;
    mat_diff.resize(subregion_num);
    for (int i = 0; i < subregion_num; i++) {    
      mat_diff.at(i).resize(subregion_num, FLT_MAX);
    }

    for (int i = 0; i < subregion_num; i++) {
      for (int j = i + 1; j < subregion_num; j++) {
        float tmp = diffSubRegion(f.at(i), f.at(j), labels.at(t), N, t);
        mat_diff.at(i).at(j) = tmp;
        mat_diff.at(j).at(i) = tmp;
      }
    }

    if (subregion_num > 1) {
      for (int i = 0; i < subregion_num; i++) {
        vector<float>::iterator tmp = std::min_element(mat_diff.at(i).begin(), mat_diff.at(i).end());
        if (*tmp > max_diff) {
          max_diff = *tmp;
          v1 = TemporalVertex(t, i);
          v2 = TemporalVertex(t, std::distance(mat_diff.at(i).begin(), tmp));
        }
      }
    }
  }

  //cout << v1.frame << " " << v1.label << endl;
  //cout << v2.frame << " " << v2.label << endl;

  return max_diff;
}

/**
 * @brief 部分領域間の相違度の計算
 * @param[in] f1  部分領域1の特徴量
 * @param[in] f2  部分領域2の特徴量
 * @param[in] label 部分領域のラベル
 * @param[in] N   最近点対の数
 * @param[in] t   フレーム番号
 * @return  部分領域間の相違度
 */
float SubRegionMatching::diffSubRegion(Feature f1, Feature f2, Mat label, int N, int t)
{
  // 変数宣言
  int width = label.cols;
  int height = label.rows;
  int l;
  // 距離の計算用
//  vector<Point> pt1(N, Point(0, 0)), pt2(N, Point(0, 0));
  vector<Point3f> pt1_3d(N, Point3f(0, 0, 0)), pt2_3d(N, Point3f(0, 0, 0));

  vector<float> dist(N, FLT_MAX);
//  vector<vector<Point>> min_point(2);
  vector<vector<Point3f>> min_point3d(2);
  float d;

  char name[80];
  sprintf(name, "./data/depth-0%004d.bmp", t + start);
  Mat depth = imread(name, CV_LOAD_IMAGE_GRAYSCALE);

  // 部分領域の画素を探索
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      l = label.at<int>(y, x);

      if (l == f1.label) {
//        min_point.at(0).push_back(Point(x, y));
        d = depth.at<uchar>(y, x);
        min_point3d.at(0).push_back(
          depth2World(x, y, luminance2Depth(d), width, height));
      }

      if (l == f2.label) {
//        min_point.at(1).push_back(Point(x, y));
        d = depth.at<uchar>(y, x);
        min_point3d.at(1).push_back(
          depth2World(x, y, luminance2Depth(d), width, height));
      }
    }
  }

  // 各部分領域間の最短距離を計算
//  vector<Point> &bpY = min_point.at(0);
//  vector<Point> &bpX = min_point.at(1);

  vector<Point3f> &bpY3d = min_point3d.at(0);
  vector<Point3f> &bpX3d = min_point3d.at(1);

  //// 初期化
  //for (int k = 0; k < N; k++) {
  //  //pt1[k] = Point(0, 0);
  //  //pt2[k] = Point(0, 0);
  //  dist[k] = FLT_MAX;

  //  pt1_3d[k] = Point3f(0, 0, 0);
  //  pt2_3d[k] = Point3f(0, 0, 0);
  //}

  // ３次元位置の比較
  for (int i = 0; i < bpY3d.size(); i++) {
//    Point &comp1 = bpY.at(i);          
    Point3f &comp1_3d = bpY3d.at(i);

    for (int j = 0; j < bpX3d.size(); j++) {
//      Point &comp2 = bpX.at(j);
      Point3f &comp2_3d = bpX3d.at(j);
      float comp3d = distanceCentroid3d(comp1_3d, comp2_3d);

      for (int k = 0; k < N; k++) {
        if (dist[k] > comp3d && comp3d != 0.0) {
          // 最短距離の画素の順番をずらす
          for (int l = N - 1; l > k; l--) {
            dist[l] = dist[l - 1];
            //pt1[l] = pt1[l - 1];
            //pt2[l] = pt2[l - 1];
            pt1_3d[l] = pt1_3d[l - 1];
            pt2_3d[l] = pt2_3d[l - 1];
          }

          dist[k] = comp3d;

          //pt1[k] = comp1;
          //pt2[k] = comp2;

          pt1_3d[k] = comp1_3d;
          pt2_3d[k] = comp2_3d;

          break;
        }
      }
    }
  }

  // 中央値の決定
  int median;
  if (N % 2 != 0) {
    median = cvFloor(N / 2.0);
  }
  else {
    median = ((N / 2) + (N / 2 - 1)) / 2.0;
  }

  return dist.at(median);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
///////////////////////////////////////////////////////////////////////////////
/**
 * @brief 双方向のエッジを生成（不要な処理）
 */
void SubRegionMatching::bidirectionalEdgeConnection(vector<TemporalEdge> &fedges, 
  vector<TemporalEdge> &bedges, Feature &f, vector<Features> &f_vec, const bool time,
  vector<Mat> &labels)
{
  int frame = f.num - start;
  int width;
  int height;

  // 前フレーム
  if (!time) {
    if (frame > 0) {
      width = labels.at(frame).cols;
      height = labels.at(frame).rows;

      Features &prev = f_vec.at(frame - 1);

      // 重なり合う部分領域の探索と重なっている画素数を計算
      vector<int> overlap(prev.size());
      Mat &current = labels.at(frame);
      Mat &previous = labels.at(frame - 1);
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          int l = previous.at<int>(y, x);

          int tmpx = min((int)(x + prev.at(l).flow2d.x), width - 1);
          tmpx = max(0, tmpx);
          int tmpy = min((int)(y + prev.at(l).flow2d.y), height - 1);
          tmpy = max(0, tmpy);

          if (current.at<int>(tmpy, tmpx) == f.label) {
            overlap.at(l)++;
          }
        }
      }

      for (int i = 0; i < prev.size(); i++) {
        if (overlap.at(i) == 0) {
          continue;
        }

        TemporalEdge fe, be;
        fe.p.frame = frame - 1;
        fe.p.label = prev.at(i).label;
        fe.n.frame = frame;
        fe.n.label = f.label;

        be.n.frame = frame - 1;
        be.n.label = prev.at(i).label;
        be.p.frame = frame;
        be.p.label = f.label;

        // 既に存在するエッジか探索
        if (findTemporalEdge(fedges, fe)) {
          continue;
        }

        // 重みの計算
        fe.w = be.w = weightOverlagEdge(prev.at(i), f, overlap.at(i));

        if (fe.w < w_threshold) {
          fedges.push_back(fe);
          bedges.push_back(be);

          // 再帰処理
          bidirectionalEdgeConnection(fedges, bedges, prev.at(i), f_vec, false, labels);
        }
      }
    }
  }
  // 次フレーム
  else {
    if (frame + 1 < f_vec.size()) {
      width = labels.at(frame).cols;
      height = labels.at(frame).rows;

      Features &next = f_vec.at(frame + 1);

      // 重なり合う部分領域の探索と重なっている画素数を計算
      vector<int> overlap(next.size());
      Mat &current = labels.at(frame);
      Mat &nexto = labels.at(frame + 1);
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          if (current.at<int>(y, x) == f.label) {
            int tmpx = min((int)(x + f.flow2d.x), width - 1);
            tmpx = max(0, tmpx);
            int tmpy = min((int)(y + f.flow2d.y), height - 1);
            tmpy = max(0, tmpy);

            int l = nexto.at<int>(tmpy, tmpx);
            overlap.at(l)++;
          }
        }
      }

      for (int i = 0; i < next.size(); i++) {
        if (overlap.at(i) == 0) {
          continue;
        }
        TemporalEdge fe, be;
        fe.p.frame = frame;
        fe.p.label = f.label;
        fe.n.frame = frame + 1;
        fe.n.label = next.at(i).label;

        be.n.frame = frame;
        be.n.label = f.label;
        be.p.frame = frame + 1;
        be.p.label = next.at(i).label;

        // 既に存在するエッジか探索
        if (findTemporalEdge(fedges, fe)) {
          continue;
        }

        // 重みの計算
        fe.w = be.w = weightOverlagEdge(f, next.at(i), overlap.at(i));

        if (fe.w < w_threshold) {
          fedges.push_back(fe);
          bedges.push_back(be);

          // 再帰処理
          bidirectionalEdgeConnection(fedges, bedges, next.at(i), f_vec, true, labels);
        }
      }
    }
  }
}

#endif

//// 双方向でのマッチング処理
//void SubRegionMatching::bidirectionalSubRegionMatching(TemporalSubRegion &fregion, 
//    TemporalSubRegion &bregion, vector<Features> &f_vec, vector<Mat> labels)
//{
//  // グラフの構築
//  vector<TemporalEdge> fedges, bedges;
//  int size = f_vec.size();
//  for (int i = 0; i < size; i++) {
//    for (int j = 0; j < f_vec.at(i).size(); j++) {
//      Feature &p = f_vec.at(i).at(j);
//      if (p.norm3d > norm_threshold) {
//        // 前フレーム
//        bidirectionalEdgeConnection(fedges, bedges, p, f_vec, false, labels);
//        // 次フレーム
//        bidirectionalEdgeConnection(fedges, bedges, p, f_vec, true, labels);
//      }
//    }
//  }
//
//  // 部分領域の対応付け
//  int num_edges = fedges.size();
//  cout << "edge num : " << num_edges << endl;
//
//  vector<int> num_vertices(f_vec.size());
//  for (int i = 0; i < f_vec.size(); i++) {
//    num_vertices.at(i) = f_vec.at(i).size();
//  }
//
//  fregion = TemporalSubRegion(f_vec.size(), num_vertices, f_vec);
//  bregion = TemporalSubRegion(f_vec.size(), num_vertices, f_vec);
//
//  // 重み順にエッジをソート
//  sort(fedges.begin(), fedges.end());
//  sort(bedges.begin(), bedges.end());
//
//  for (int i = 0; i < num_edges; i++) {
//    TemporalEdge &pedge = fedges.at(i);
//
//    // 部分領域の対応付け
//    int f_prev = 0, l_prev = 0, r_prev = 0, f_next = 0, l_next = 0, r_next = 0, buf;
//    fregion.find(pedge.p.frame, pedge.p.label, f_prev, l_prev, r_prev, buf);
//    fregion.find(pedge.n.frame, pedge.n.label, f_next, l_next, r_next, buf);
//
//    if (r_next == 0 && pedge.w < w_threshold) {
//      // 対応付け
//      fregion.join(f_prev, l_prev, f_next, l_next);
//    }
//
//    // 逆方向でも同様の処理
//    pedge = bedges.at(i);
//
//    bregion.find(pedge.p.frame, pedge.p.label, f_prev, l_prev, r_prev, buf);
//    bregion.find(pedge.n.frame, pedge.n.label, f_next, l_next, r_next, buf);
//
//    if (r_next == 0 && pedge.w < w_threshold) {
//      // 対応付け
//      bregion.join(f_prev, l_prev, f_next, l_next);
//    }
//  }
//}

//RegionSequences SubRegionMatching::regionToSequence2(TemporalSubRegion &fregion,
//  TemporalSubRegion &bregion,
//  vector<Features> &f_vec, const int min_frame, const float min_move)
//{
//  // 領域系列の決定
//  vector<RegionSequence> fregion_seq, bregion_seq;
//  int frame, label, r_next, r_prev;
//  int frame_num = f_vec.size();
//  int region_num;
//  int fseq_num, bseq_num;
//
//  for (int i = 0; i < frame_num; i++) {
//    region_num = f_vec.at(i).size();
//    for (int j = 0; j < region_num; j++) {
//      fregion.find(i, j, frame, label, r_next, r_prev);
//
//      // 対応付けされているかを取得
//      if ((r_next != 0 || r_prev != 0) && fregion.getNorm(frame, label)) {          
//        bool flag = false;
//
//        // 領域系列へ追加
//        int size = fregion_seq.size();
//        for (int k = 0; k < size; k++) {
//          if (fregion_seq.at(k).getLabel() == TemporalVertex(frame, label)) {
//            fregion_seq.at(k).setFeature(i, f_vec.at(i).at(j));
//
//            flag = true;
//            break;
//          }
//        }
//
//        // 新しい領域系列を作成
//        if (!flag) {
//          RegionSequence tmp(TemporalVertex(frame, label), frame_num);
//          tmp.setFeature(i, f_vec.at(i).at(j));
//          fregion_seq.push_back(tmp);
//        }
//      }
//    }
//  }
//
//  /////////////////////////
//  //fseq_num = fregion_seq.size();
//  //for (int i = 0; i < fseq_num; i++) {
//  //  fregion_seq.at(i).calcNorm();
//  //}
//
//  //// 短い系列，移動の小さい系列を除去
//  //for (int i = 0; i < fseq_num; i++) {
//  //  if (fregion_seq.at(i).getLength() < min_frame || 
//  //    fregion_seq.at(i).getNorm() < min_move) 
//  //  {
//  //    fregion_seq.erase(fregion_seq.begin() + i);
//  //    fseq_num--;
//  //    i--;
//  //  }
//  //}
//
//  //return fregion_seq;
//  /////////////////////////
//
//
//  for (int i = 0; i < frame_num; i++) {
//    region_num = f_vec.at(i).size();
//    for (int j = 0; j < region_num; j++) {
//      bregion.find(i, j, frame, label, r_next, r_prev);
//
//      // 対応付けされているかを取得
//      if ((r_next != 0 || r_prev != 0) && bregion.getNorm(frame, label)) {          
//        bool flag = false;
//
//        // 領域系列へ追加
//        int size = bregion_seq.size();
//        for (int k = 0; k < size; k++) {
//          if (bregion_seq.at(k).getLabel() == TemporalVertex(frame, label)) {
//            bregion_seq.at(k).setFeature(i, f_vec.at(i).at(j));
//
//            flag = true;
//            break;
//          }
//        }
//
//        // 新しい領域系列を作成
//        if (!flag) {
//          RegionSequence tmp(TemporalVertex(frame, label), frame_num);
//          tmp.setFeature(i, f_vec.at(i).at(j));
//          bregion_seq.push_back(tmp);
//        }
//      }
//    }
//  }
//
//  // 逆方向の対応付け結果の領域系列のラベルを修正
//  bseq_num = bregion_seq.size();
//  for (int i = 0; i < bseq_num; i++) {
//    for (int j = 0; j < frame_num; j++) {
//      if (bregion_seq.at(i).getFeatures(j).size() != 0) {
//        bregion_seq.at(i).setLabel(TemporalVertex(bregion_seq.at(i).getFeatures(j).at(0).num - start, 
//          bregion_seq.at(i).getFeatures(j).at(0).label));
//        break;
//      }
//    }
//  }
//
//  /////////////////////////
//  bseq_num = bregion_seq.size();
//  for (int i = 0; i < bseq_num; i++) {
//    bregion_seq.at(i).calcNorm();
//  }
//
//  // 短い系列，移動の小さい系列を除去
//  for (int i = 0; i < bseq_num; i++) {
//    if (bregion_seq.at(i).getLength() < min_frame || 
//      bregion_seq.at(i).getNorm() < min_move) 
//    {
//      bregion_seq.erase(bregion_seq.begin() + i);
//      bseq_num--;
//      i--;
//    }
//  }
//
//  return bregion_seq;
//  /////////////////////////
//
//
//  fseq_num = fregion_seq.size();
//
//  // 双方向の領域系列の和を算出
//  vector<RegionSequence> region_seq;
//  for (int i = 0; i < fseq_num; i++) {
//    for (int j = 0; j < bseq_num; j++) {
//      if (fregion_seq.at(i).getLabel() == bregion_seq.at(j).getLabel()) {
//        region_seq.push_back(fregion_seq.at(i) + bregion_seq.at(j));
//        fregion_seq.erase(fregion_seq.begin() + i);
//        fseq_num--;
//        i--;
//
//        bregion_seq.erase(bregion_seq.begin() + j);
//        bseq_num--;
//        j--;
//
//        break;
//      }
//    }
//  }
//  // 余った領域系列をリストに追加
//  for (int i = 0; i < fseq_num; i++) {
//    region_seq.push_back(fregion_seq.at(i));
//  }
//  for (int i = 0; i < bseq_num; i++) {
//    region_seq.push_back(bregion_seq.at(i));
//  }
//
//  int seq_num = region_seq.size();
//  for (int i = 0; i < seq_num; i++) {
//    region_seq.at(i).calcNorm();
//  }
//
//  // 短い系列，移動の小さい系列を除去
//  for (int i = 0; i < seq_num; i++) {
//    if (region_seq.at(i).getLength() < min_frame || 
//      region_seq.at(i).getNorm() < min_move) 
//    {
//      region_seq.erase(region_seq.begin() + i);
//      seq_num--;
//      i--;
//    }
//  }
//
//  return region_seq;
//}

///////////////////////////////////////////////////////////////////////////////