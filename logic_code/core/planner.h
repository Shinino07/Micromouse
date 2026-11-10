#pragma once
#include <cstdint>
#include "maze.h"

namespace mm {

struct CellMask {
  uint32_t w[8] = {};
  void clear() { for (auto& x : w) x = 0; }
  void set(uint8_t c) { w[c >> 5] |= 1u << (c & 31); }
  bool has(uint8_t c) const { return w[c >> 5] & (1u << (c & 31)); }
  bool empty() const { for (auto x : w) if (x) return false; return true; }
};

CellMask goal_mask();
CellMask single_mask(int x, int y);

constexpr uint16_t FLOOD_INF = 0xFFFF;

// BFS khoang cach (so o) tu tap dich; chua biet = mo neu unknown_open
void flood(const Maze& m, const CellMask& targets, bool unknown_open, uint16_t dist[cfg::N * cfg::N]);

// Huong di tiep khi do duong; tra ve false neu khong toi duoc dich
bool explore_step(const Maze& m, const uint16_t dist[], int x, int y, Dir heading, Dir& out);

// Duong di theo o, co trong so thoi gian (thang lien tiep re, zigzag = cheo re)
struct Route {
  uint16_t n = 0;                    // so o, cell[0] = o dang dung
  uint8_t  cell[cfg::N * cfg::N + 2];
  Act      act[cfg::N * cfg::N + 2]; // act[0] = S (sau khi xoay tai cho)
  Dir      first_dir = NORTH;        // huong ra khoi o dau
  uint32_t cost = 0;
};

bool find_route(const Maze& m, uint8_t start, Dir heading, const CellMask& targets,
                bool unknown_open, Route& out);

}  // namespace mm
