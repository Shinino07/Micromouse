#pragma once
#include <cstdint>
#include "maze.h"

// Dich vu nen tang cho vong lap chinh (port STM32 hoac mo phong cai dat)
namespace mm::hw {

uint32_t millis();
bool btn_mode();    // true = dang nhan
bool btn_start();
bool load_maze(Maze& m);   // false neu flash chua co ban do hop le
void save_maze(const Maze& m);
void erase_maze();
void log(const char* fmt, ...);
void system_reset();   // khoi dong lai chip (ban do trong flash van con)

}  // namespace mm::hw
