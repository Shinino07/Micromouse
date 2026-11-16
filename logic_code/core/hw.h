#pragma once
#include <cstdint>
#include "maze.h"

namespace mm::hw {

uint32_t millis();
bool btn_mode();
bool btn_start();
bool load_maze(Maze& m);
void save_maze(const Maze& m);
void erase_maze();
void log(const char* fmt, ...);
void system_reset();

}
