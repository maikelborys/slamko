// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// vpr_recall_diag — P-B step 1 (MASTER_PLAN §8): the per-KF VPR recall
// diagnostic on the magistrale start↔end bridge. The Atlas bottleneck is
// RECALL, not merge machinery (CudaSIFT-SLAM: +70% mapped frames purely from
// recall) — so before wiring relocalization into the live chain, measure
// whether EigenPlaces per-KF retrieval can recognize the start room from the
// RETURN keyframes against the full map as distractor set.
//
// Setup: DB = all VPR keyframes of the first --db-first submaps (the start
// room); queries = all VPR keyframes of the last --query-last submaps (the
// return); distractors = everything in between. For each query we rank ALL
// non-query keyframes by cosine and ask: at what rank does the first start-room
// keyframe appear? R@k over the query set is the recall number that decides
// EigenPlaces-sufficient vs SALAD/CosPlace fallback. Margins (best DB cosine vs
// best distractor cosine) tell whether failures are a hair (verification will
// fix) or a gulf (descriptor swap needed).
//
// Per-query CSV (--csv) so failures are inspectable frame-by-frame; absolute
// cosines are expected LOW (~0.25 domain gap, PLAN_VPR_RELOC) — only the
// RANKING is meaningful.
//
// Two split modes:
//   submap-index (default): DB = first --db-first submaps, queries = last
//     --query-last submaps. CAVEAT (learned on magistrale1): the return-room
//     keyframes BEFORE the query submaps land in the distractor set and absorb
//     the matches — prefer the time split when GT room windows are known.
//   time (--db-tmax T1 --query-tmin T2): DB/correct = KFs with t<=T1 (the
//     start room per GT), queries = KFs with t>=T2 (the return), distractors =
//     everything in between (the corridor trek). This is the honest
//     cross-session reloc question: can a return frame retrieve the START room
//     against the whole outbound map?
//
//   vpr_recall_diag --map <smap_dir> [--db-first 2] [--query-last 2]
//                   [--db-tmax T] [--query-tmin T] [--csv out.csv] [--topk 10]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "slamko_core/submap_io.hpp"

namespace {

struct Entry {
  std::size_t submap_idx;     // index into the loaded vector (chronological)
  std::uint64_t submap_id;
  std::size_t kf_idx;
  double t;
  Eigen::VectorXf g;          // L2-normalized global descriptor
};

}  // namespace

int main(int argc, char** argv) {
  std::string map_dir, csv_path;
  std::size_t db_first = 2, query_last = 2, topk = 10;
  double db_tmax = 0.0, query_tmin = 0.0;  // both >0 => time-split mode
  for (int i = 1; i < argc - 1; ++i) {
    if (!std::strcmp(argv[i], "--map")) map_dir = argv[++i];
    else if (!std::strcmp(argv[i], "--csv")) csv_path = argv[++i];
    else if (!std::strcmp(argv[i], "--db-first")) db_first = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--query-last")) query_last = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--db-tmax")) db_tmax = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--query-tmin")) query_tmin = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--topk")) topk = std::atoi(argv[++i]);
  }
  const bool time_split = db_tmax > 0.0 && query_tmin > 0.0;
  if (map_dir.empty()) {
    std::fprintf(stderr, "usage: vpr_recall_diag --map <smap_dir> [--db-first N] "
                         "[--query-last M] [--csv out.csv] [--topk K]\n");
    return 2;
  }

  std::vector<slamko::SubMap> maps;
  if (!slamko::loadSubMaps(maps, map_dir)) {
    std::fprintf(stderr, "FAIL: loadSubMaps('%s')\n", map_dir.c_str());
    return 1;
  }

  // Cross-map mode (--map2): DB = ALL keyframes of --map, queries = ALL
  // keyframes of --map2, no distractors. Answers "can session B's frames
  // retrieve session A's map at all?" — the retrieval half of cross-session.
  std::string map2_dir;
  for (int i = 1; i < argc - 1; ++i)
    if (!std::strcmp(argv[i], "--map2")) map2_dir = argv[i + 1];
  if (!map2_dir.empty()) {
    std::vector<slamko::SubMap> maps2;
    if (!slamko::loadSubMaps(maps2, map2_dir)) {
      std::fprintf(stderr, "FAIL: loadSubMaps('%s')\n", map2_dir.c_str());
      return 1;
    }
    std::vector<Eigen::VectorXf> dbv;
    std::vector<std::pair<std::size_t, std::size_t>> dbidx;
    for (std::size_t s = 0; s < maps.size(); ++s)
      for (std::size_t k = 0; k < maps[s].kf_obs.size(); ++k)
        if (maps[s].kf_obs[k].hasGlobalDescriptor()) {
          dbv.push_back(maps[s].kf_obs[k].global_descriptor);
          dbidx.emplace_back(s, k);
        }
    Eigen::MatrixXf DB(dbv.size(), dbv[0].size());
    for (std::size_t i = 0; i < dbv.size(); ++i) DB.row(i) = dbv[i].transpose();
    std::printf("cross-map: DB(%s)=%zu KF | queries(%s)\n", map_dir.c_str(),
                dbv.size(), map2_dir.c_str());
    std::FILE* csv = csv_path.empty() ? nullptr : std::fopen(csv_path.c_str(), "w");
    if (csv) std::fprintf(csv, "q_submap,q_kf,best_cos,best_db_submap,best_db_kf\n");
    std::vector<float> best_all;
    for (std::size_t s = 0; s < maps2.size(); ++s) {
      float best_sm = -1;
      std::size_t bi = 0;
      for (std::size_t k = 0; k < maps2[s].kf_obs.size(); ++k) {
        if (!maps2[s].kf_obs[k].hasGlobalDescriptor()) continue;
        Eigen::VectorXf cs = DB * maps2[s].kf_obs[k].global_descriptor;
        int arg = 0;
        const float b = cs.maxCoeff(&arg);
        best_all.push_back(b);
        if (b > best_sm) { best_sm = b; bi = (std::size_t)arg; }
        if (csv)
          std::fprintf(csv, "%zu,%zu,%.4f,%zu,%zu\n", s, k, b, dbidx[arg].first,
                       dbidx[arg].second);
      }
      std::printf("  q submap %zu: best cos %.3f -> DB submap %zu kf %zu\n", s,
                  best_sm, dbidx[bi].first, dbidx[bi].second);
    }
    std::sort(best_all.begin(), best_all.end());
    std::printf("best-cos percentiles: p50=%.3f p90=%.3f max=%.3f\n",
                best_all[best_all.size() / 2], best_all[(best_all.size() * 9) / 10],
                best_all.back());
    if (csv) std::fclose(csv);
    return 0;
  }
  if (maps.size() < db_first + query_last + 1) {
    std::fprintf(stderr, "FAIL: %zu submaps < db_first+query_last+1\n", maps.size());
    return 1;
  }

  // Flatten per-KF VPR entries, tagged by role.
  std::vector<Entry> db, mid, queries;
  for (std::size_t s = 0; s < maps.size(); ++s) {
    const auto& m = maps[s];
    for (std::size_t k = 0; k < m.kf_obs.size(); ++k) {
      if (!m.kf_obs[k].hasGlobalDescriptor()) continue;
      Entry e{s, m.id, k, k < m.keyframes.size() ? m.keyframes[k].timestamp : 0.0,
              m.kf_obs[k].global_descriptor};
      if (time_split) {
        if (e.t <= db_tmax) db.push_back(std::move(e));
        else if (e.t >= query_tmin) queries.push_back(std::move(e));
        else mid.push_back(std::move(e));
      } else {
        if (s < db_first) db.push_back(std::move(e));
        else if (s >= maps.size() - query_last) queries.push_back(std::move(e));
        else mid.push_back(std::move(e));
      }
    }
  }
  if (time_split)
    std::printf("map=%s submaps=%zu | TIME split: DB(t<=%.1f)=%zu KF | "
                "distractors=%zu KF | queries(t>=%.1f)=%zu KF\n",
                map_dir.c_str(), maps.size(), db_tmax, db.size(), mid.size(),
                query_tmin, queries.size());
  else
    std::printf("map=%s submaps=%zu | DB(start, first %zu submaps)=%zu KF | "
                "distractors=%zu KF | queries(return, last %zu submaps)=%zu KF\n",
                map_dir.c_str(), maps.size(), db_first, db.size(), mid.size(),
                query_last, queries.size());
  {
    double tmin = 1e18, tmax = 0.0;
    for (const auto* v : {&db, &mid, &queries})
      for (const auto& e : *v) { tmin = std::min(tmin, e.t); tmax = std::max(tmax, e.t); }
    std::printf("KF time range: %.3f .. %.3f (span %.1f s)\n", tmin, tmax, tmax - tmin);
  }
  if (db.empty() || queries.empty()) {
    std::fprintf(stderr, "FAIL: empty DB or query set\n");
    return 1;
  }

  // Stack DB + distractors into matrices for one-shot cosine evaluation.
  const int D = static_cast<int>(db[0].g.size());
  auto stack = [D](const std::vector<Entry>& v) {
    Eigen::MatrixXf M(static_cast<int>(v.size()), D);
    for (std::size_t i = 0; i < v.size(); ++i) M.row(static_cast<int>(i)) = v[i].g.transpose();
    return M;
  };
  const Eigen::MatrixXf DBm = stack(db), MIDm = mid.empty() ? Eigen::MatrixXf(0, D) : stack(mid);

  std::FILE* csv = csv_path.empty() ? nullptr : std::fopen(csv_path.c_str(), "w");
  if (csv)
    std::fprintf(csv, "q_submap,q_kf,t,best_db_cos,best_db_submap,best_db_kf,"
                      "best_mid_cos,best_mid_submap,margin,rank_first_db\n");

  std::vector<std::size_t> rank_first_db(queries.size());
  std::size_t r_at[3] = {0, 0, 0};  // R@1, R@5, R@topk
  double sum_margin = 0.0;
  for (std::size_t qi = 0; qi < queries.size(); ++qi) {
    const auto& q = queries[qi];
    const Eigen::VectorXf cd = DBm * q.g;                        // cos vs DB
    const Eigen::VectorXf cm = MIDm.rows() ? (MIDm * q.g).eval() // cos vs distractors
                                           : Eigen::VectorXf();
    int best_db_i = 0;
    const float best_db = cd.maxCoeff(&best_db_i);
    int best_mid_i = 0;
    const float best_mid = cm.size() ? cm.maxCoeff(&best_mid_i) : -1.0f;

    // Rank of the first DB hit in the combined (DB + distractor) ranking =
    // 1 + number of distractors scoring above the best DB cosine.
    std::size_t above = 0;
    for (int i = 0; i < cm.size(); ++i)
      if (cm[i] > best_db) ++above;
    const std::size_t rank = 1 + above;
    rank_first_db[qi] = rank;
    if (rank <= 1) ++r_at[0];
    if (rank <= 5) ++r_at[1];
    if (rank <= topk) ++r_at[2];
    sum_margin += double(best_db) - double(best_mid);

    if (csv)
      std::fprintf(csv, "%zu,%zu,%.9f,%.4f,%llu,%zu,%.4f,%llu,%.4f,%zu\n",
                   q.submap_idx, q.kf_idx, q.t, best_db,
                   (unsigned long long)db[best_db_i].submap_id, db[best_db_i].kf_idx,
                   best_mid,
                   cm.size() ? (unsigned long long)mid[best_mid_i].submap_id : 0ull,
                   best_db - best_mid, rank);
  }
  if (csv) std::fclose(csv);

  const double n = double(queries.size());
  std::printf("R@1   = %.3f  (%zu/%zu)\n", r_at[0] / n, r_at[0], queries.size());
  std::printf("R@5   = %.3f  (%zu/%zu)\n", r_at[1] / n, r_at[1], queries.size());
  std::printf("R@%-3zu = %.3f  (%zu/%zu)\n", topk, r_at[2] / n, r_at[2], queries.size());
  std::printf("mean margin (best_db_cos - best_distractor_cos) = %+.4f\n",
              sum_margin / n);

  std::vector<std::size_t> sorted = rank_first_db;
  std::sort(sorted.begin(), sorted.end());
  std::printf("rank of first start-room hit: median=%zu p90=%zu worst=%zu\n",
              sorted[sorted.size() / 2], sorted[(sorted.size() * 9) / 10],
              sorted.back());
  return 0;
}
