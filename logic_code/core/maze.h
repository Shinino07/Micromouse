#pragma once
#include <cstdint>
#include "geometry.h"

namespace mm {

// Moi o 1 byte: bit 0..3 = tuong N E S W, bit 4..7 = da biet N E S W
class Maze {
 public:
  void reset();
  bool wall(int x, int y, Dir d) const { return cells_[cell_id(x, y)] & (1u << d); }
  bool known(int x, int y, Dir d) const { return cells_[cell_id(x, y)] & (16u << d); }
  bool open_known(int x, int y, Dir d) const { return known(x, y, d) && !wall(x, y, d); }
  // Mo cua theo che do: lac quan (chua biet = mo) hay than trong (chua biet = tuong)
  bool passable(int x, int y, Dir d, bool unknown_open) const {
    if (wall(x, y, d)) return false;
    return unknown_open || known(x, y, d);
  }
  bool visited(int x, int y) const { return (cells_[cell_id(x, y)] & 0xF0) == 0xF0; }
  // Tra ve true neu thong tin thay doi
  bool set_wall(int x, int y, Dir d, bool present);
  void forget(int x, int y, Dir d);  // ve lai trang thai chua biet (tru tuong bien)
  uint8_t raw(uint8_t c) const { return cells_[c]; }
  void set_raw(uint8_t c, uint8_t v) { cells_[c] = v; }
  uint32_t checksum() const;
  bool sane() const;

 private:
  uint8_t cells_[cfg::N * cfg::N];
};

}  // namespace mm
