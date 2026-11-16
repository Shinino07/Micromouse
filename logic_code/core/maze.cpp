#include "maze.h"

namespace mm {

bool Maze::set_wall(int x, int y, Dir d, bool present) {
  if (!in_maze(x, y)) return false;
  const int nx = x + DX[d], ny = y + DY[d];
  if (!in_maze(nx, ny)) return false;
  uint8_t& a = cells_[cell_id(x, y)];
  uint8_t& b = cells_[cell_id(nx, ny)];
  const Dir o = dir_back(d);
  const uint8_t old_a = a;
  a = static_cast<uint8_t>((a & ~(1u << d)) | (present ? (1u << d) : 0) | (16u << d));
  b = static_cast<uint8_t>((b & ~(1u << o)) | (present ? (1u << o) : 0) | (16u << o));
  return a != old_a;
}

void Maze::forget(int x, int y, Dir d) {
  const int nx = x + DX[d], ny = y + DY[d];
  if (!in_maze(x, y) || !in_maze(nx, ny)) return;
  const Dir o = dir_back(d);
  cells_[cell_id(x, y)] &= static_cast<uint8_t>(~((1u << d) | (16u << d)));
  cells_[cell_id(nx, ny)] &= static_cast<uint8_t>(~((1u << o) | (16u << o)));
}

void Maze::reset() {
  for (auto& c : cells_) c = 0;
  for (int i = 0; i < cfg::N; i++) {
    cells_[cell_id(i, 0)] |= (1u << SOUTH) | (16u << SOUTH);
    cells_[cell_id(i, cfg::N - 1)] |= (1u << NORTH) | (16u << NORTH);
    cells_[cell_id(0, i)] |= (1u << WEST) | (16u << WEST);
    cells_[cell_id(cfg::N - 1, i)] |= (1u << EAST) | (16u << EAST);
  }
  set_wall(0, 0, EAST, true);
  set_wall(0, 0, NORTH, false);
  const int gx = cfg::GOAL_X0, gy = cfg::GOAL_Y0;
  set_wall(gx, gy, EAST, false);
  set_wall(gx, gy, NORTH, false);
  set_wall(gx + 1, gy + 1, WEST, false);
  set_wall(gx + 1, gy + 1, SOUTH, false);
}

uint32_t Maze::checksum() const {
  uint32_t h = 2166136261u;
  for (uint8_t c : cells_) h = (h ^ c) * 16777619u;
  return h;
}

bool Maze::sane() const {
  for (int y = 0; y < cfg::N; y++)
    for (int x = 0; x < cfg::N; x++)
      for (int d = 0; d < 2; d++) {
        const Dir dd = static_cast<Dir>(d);
        const int nx = x + DX[dd], ny = y + DY[dd];
        if (!in_maze(nx, ny)) { if (!wall(x, y, dd)) return false; continue; }
        if (wall(x, y, dd) != wall(nx, ny, dir_back(dd))) return false;
      }
  return wall(0, 0, EAST) && !wall(0, 0, NORTH);
}

}
