#pragma once
#include <cstdint>
#include "config.h"

namespace mm {

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Dir dir_add(Dir d, int k) { return static_cast<Dir>((d + k + 8) & 3); }
inline Dir dir_left(Dir d) { return dir_add(d, -1); }
inline Dir dir_right(Dir d) { return dir_add(d, 1); }
inline Dir dir_back(Dir d) { return dir_add(d, 2); }

constexpr int8_t DX[4] = {0, 1, 0, -1};
constexpr int8_t DY[4] = {1, 0, -1, 0};

inline uint8_t cell_id(int x, int y) { return static_cast<uint8_t>(y * cfg::N + x); }
inline int cell_x(uint8_t c) { return c % cfg::N; }
inline int cell_y(uint8_t c) { return c / cfg::N; }
inline bool in_maze(int x, int y) { return x >= 0 && y >= 0 && x < cfg::N && y < cfg::N; }
inline bool is_goal(int x, int y) {
  return (x == cfg::GOAL_X0 || x == cfg::GOAL_X0 + 1) && (y == cfg::GOAL_Y0 || y == cfg::GOAL_Y0 + 1);
}
inline bool is_goal(uint8_t c) { return is_goal(cell_x(c), cell_y(c)); }

enum Act : uint8_t { ACT_S = 0, ACT_L = 1, ACT_R = 2 };

inline float heading_rad(Dir d) {
  return static_cast<float>(-static_cast<int>(d)) * 1.5707963f;
}

}
