#include "planner.h"

namespace mm {

namespace {
constexpr int NC = cfg::N * cfg::N;
constexpr int NS = NC * 4 * 3;  // (o, huong vao, hanh dong o truoc)
constexpr uint16_t NONE = 0xFFFF;

uint32_t g_cost[NS];
uint16_t g_parent[NS];
uint16_t g_heap[NS];
uint16_t g_pos[NS];
int g_heap_n;

inline uint16_t sid(uint8_t c, Dir h, Act p) { return static_cast<uint16_t>((c * 4 + h) * 3 + p); }
inline uint8_t s_cell(uint16_t s) { return static_cast<uint8_t>(s / 12); }
inline Dir s_head(uint16_t s) { return static_cast<Dir>((s / 3) & 3); }
inline Act s_prev(uint16_t s) { return static_cast<Act>(s % 3); }

void heap_swap(int a, int b) {
  const uint16_t t = g_heap[a]; g_heap[a] = g_heap[b]; g_heap[b] = t;
  g_pos[g_heap[a]] = static_cast<uint16_t>(a);
  g_pos[g_heap[b]] = static_cast<uint16_t>(b);
}
void heap_up(int i) {
  while (i > 0) {
    const int p = (i - 1) >> 1;
    if (g_cost[g_heap[p]] <= g_cost[g_heap[i]]) break;
    heap_swap(i, p); i = p;
  }
}
void heap_down(int i) {
  for (;;) {
    const int l = 2 * i + 1, r = l + 1;
    int m = i;
    if (l < g_heap_n && g_cost[g_heap[l]] < g_cost[g_heap[m]]) m = l;
    if (r < g_heap_n && g_cost[g_heap[r]] < g_cost[g_heap[m]]) m = r;
    if (m == i) break;
    heap_swap(i, m); i = m;
  }
}
void relax(uint16_t s, uint32_t c, uint16_t parent) {
  if (c >= g_cost[s]) return;
  const bool in_heap = g_cost[s] != 0xFFFFFFFFu && g_pos[s] != NONE;
  g_cost[s] = c;
  g_parent[s] = parent;
  if (!in_heap) {
    g_heap[g_heap_n] = s; g_pos[s] = static_cast<uint16_t>(g_heap_n); g_heap_n++;
  }
  heap_up(g_pos[s]);
}
uint16_t heap_pop() {
  const uint16_t s = g_heap[0];
  g_heap_n--;
  if (g_heap_n > 0) { g_heap[0] = g_heap[g_heap_n]; g_pos[g_heap[0]] = 0; heap_down(0); }
  g_pos[s] = NONE;
  return s;
}

uint16_t step_cost(Act prev, Act a) {
  if (a == ACT_S) return prev == ACT_S ? cfg::COST_F_NEXT : cfg::COST_F_FIRST;
  if (prev == ACT_S) return cfg::COST_TURN;
  return prev == a ? cfg::COST_SAME_TURN : cfg::COST_ZIGZAG;
}
}  // namespace

CellMask goal_mask() {
  CellMask m;
  for (int dy = 0; dy < 2; dy++)
    for (int dx = 0; dx < 2; dx++) m.set(cell_id(cfg::GOAL_X0 + dx, cfg::GOAL_Y0 + dy));
  return m;
}

CellMask single_mask(int x, int y) {
  CellMask m;
  m.set(cell_id(x, y));
  return m;
}

void flood(const Maze& m, const CellMask& targets, bool unknown_open, uint16_t dist[]) {
  uint8_t queue[NC];
  int head = 0, tail = 0;
  for (int c = 0; c < NC; c++) {
    dist[c] = FLOOD_INF;
    if (targets.has(static_cast<uint8_t>(c))) { dist[c] = 0; queue[tail++] = static_cast<uint8_t>(c); }
  }
  while (head < tail) {
    const uint8_t c = queue[head++];
    const int x = cell_x(c), y = cell_y(c);
    for (int d = 0; d < 4; d++) {
      if (!m.passable(x, y, static_cast<Dir>(d), unknown_open)) continue;
      const uint8_t n = cell_id(x + DX[d], y + DY[d]);
      if (dist[n] != FLOOD_INF) continue;
      dist[n] = static_cast<uint16_t>(dist[c] + 1);
      queue[tail++] = n;
    }
  }
}

bool explore_step(const Maze& m, const uint16_t dist[], int x, int y, Dir heading, Dir& out) {
  static const int ORDER[4] = {0, -1, 1, 2};  // thang, trai, phai, quay lui
  uint16_t best = FLOOD_INF;
  int best_rank = 99;
  for (int k = 0; k < 4; k++) {
    const Dir d = dir_add(heading, ORDER[k]);
    if (!m.passable(x, y, d, true)) continue;
    const uint16_t v = dist[cell_id(x + DX[d], y + DY[d])];
    if (v == FLOOD_INF) continue;
    // uu tien o chua tham khi bang diem
    const int rank = k + (m.visited(x + DX[d], y + DY[d]) ? 4 : 0);
    if (v < best || (v == best && rank < best_rank)) { best = v; best_rank = rank; out = d; }
  }
  return best != FLOOD_INF && best < dist[cell_id(x, y)];
}

bool find_route(const Maze& m, uint8_t start, Dir heading, const CellMask& targets,
                bool unknown_open, Route& out) {
  for (int s = 0; s < NS; s++) { g_cost[s] = 0xFFFFFFFFu; g_pos[s] = NONE; g_parent[s] = NONE; }
  g_heap_n = 0;
  out.n = 0;

  const int sx = cell_x(start), sy = cell_y(start);
  for (int k = 0; k < 4; k++) {
    const Dir e = dir_add(heading, k);
    if (!m.passable(sx, sy, e, unknown_open)) continue;
    const uint16_t pivot = k == 0 ? 0 : (k == 2 ? cfg::COST_PIVOT180 : cfg::COST_PIVOT90);
    const uint8_t n = cell_id(sx + DX[e], sy + DY[e]);
    relax(sid(n, e, ACT_S), pivot + cfg::COST_F_FIRST, NONE);
  }

  uint16_t found = NONE;
  while (g_heap_n > 0) {
    const uint16_t s = heap_pop();
    const uint8_t c = s_cell(s);
    if (targets.has(c)) { found = s; break; }
    const Dir h = s_head(s);
    const Act p = s_prev(s);
    const int x = cell_x(c), y = cell_y(c);
    for (int a = 0; a < 3; a++) {
      const Dir e = a == ACT_S ? h : (a == ACT_L ? dir_left(h) : dir_right(h));
      if (!m.passable(x, y, e, unknown_open)) continue;
      const uint8_t n = cell_id(x + DX[e], y + DY[e]);
      relax(sid(n, e, static_cast<Act>(a)), g_cost[s] + step_cost(p, static_cast<Act>(a)), s);
    }
  }
  if (found == NONE) return false;

  // Truy vet: moi trang thai (o, huong vao, act o truoc)
  uint16_t chain[NC + 2];
  int len = 0;
  for (uint16_t s = found; s != NONE; s = g_parent[s]) chain[len++] = s;
  out.n = static_cast<uint16_t>(len + 1);
  out.cell[0] = start;
  out.act[0] = ACT_S;
  out.first_dir = s_head(chain[len - 1]);
  for (int i = 0; i < len; i++) {
    const uint16_t s = chain[len - 1 - i];
    out.cell[i + 1] = s_cell(s);
    // act cua o i+1 = "act o truoc" cua trang thai ke tiep; o cuoi di thang
    out.act[i + 1] = (i + 1 < len) ? s_prev(chain[len - 2 - i]) : ACT_S;
  }
  out.cost = g_cost[found];
  return true;
}

}  // namespace mm
