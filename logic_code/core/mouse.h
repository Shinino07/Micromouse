#pragma once
#include <cstdint>
#include "motion.h"

namespace mm {

enum class Mode : uint8_t {
  AUTO,           // 3 luot lien tiep: do duong -> nhanh an toan -> nhanh nhat
  EXPLORE,
  FAST_SAFE,
  FAST_MAX,
  TEST_STRAIGHT,  // chay thang 5 o (do xung/mm, chinh KV)
  TEST_PIVOT,     // xoay tai cho 10 vong (do TRACK, GYRO_SCALE)
  TEST_SENSORS,   // in IR + nhan tuong qua UART
  ERASE,          // xoa ban do
  COUNT
};

class Mouse {
 public:
  void init();
  void loop();                                  // vong lap chinh
  void tick(const TickIn& in, TickOut& out);    // ngat 1 kHz

  void start(Mode m);                           // nhu bam BTN_Start voi che do m
  bool idle() const { return st_ == St::MENU; }
  const Maze& maze() const { return maze_; }
  Motion& motion() { return motion_; }
  uint32_t goal_time_ms(int run) const { return run < 4 ? goal_ms_[run] : 0; }
  int runs_done() const { return runs_done_; }
  bool had_fault() const { return fault_seen_; }

 private:
  enum class St : uint8_t { MENU, ARMED, RUN, REST };
  enum class Run : uint8_t { NONE, EXPLORE, FAST, GOAL_CAL, RETURN, TEST };
  enum class Phase : uint8_t { TO_GOAL, IN_GOAL, SEARCH, HOME };

  void menu();
  void begin_run(Mode m);
  void end_run();
  void handle_fault();

  void explore_begin(Phase ph, bool from_back);
  void explore_update();
  void explore_push(Dir d);
  bool explore_target(CellMask& tgt);
  void finish_home();

  bool fast_begin(const cfg::RunParams& rp);
  bool return_begin();
  void feed_moves();
  bool build_moves(const Route& r, Dir heading, float offset, const cfg::RunParams& rp, uint8_t end_cell, Dir end_head);
  bool path_proven();
  void update_wall(int x, int y, Dir d, bool present);

  Maze maze_;
  Motion motion_;
  St st_ = St::MENU;
  Run run_ = Run::NONE;
  Phase phase_ = Phase::TO_GOAL;
  Mode mode_ = Mode::AUTO;
  Mode plan_[3] = {};
  int plan_n_ = 0, plan_i_ = 0;

  int cx_ = 0, cy_ = 0;
  Dir head_ = NORTH;
  uint8_t first_goal_ = 0;
  bool finishing_ = false;
  bool armed_cal_ = false;
  bool at_center_ = false;   // robot dang o tam o xuat phat (luot 2, 3)
  uint32_t t_run_ = 0, t_state_ = 0;
  uint32_t goal_ms_[4] = {};
  int runs_done_ = 0;
  bool fault_seen_ = false;

  static constexpr int MAXM = 200;
  Move moves_[MAXM];
  int moves_n_ = 0, moves_fed_ = 0;
  Dir end_head_ = NORTH;
  uint8_t end_cell_ = 0;

  // nut bam co chong doi: chi doi trang thai khi muc doc on dinh >= BTN_STABLE_MS
  struct Btn {
    bool raw = false, down = false;
    bool pressed = false, released = false;  // su kien trong lan poll nay
    bool skip_release = false;               // nha cua lan bam da dung de dung robot -> bo qua
    uint32_t t_raw = 0, t_down = 0;
  };
  static void poll_btn(Btn& b, bool raw, uint32_t now);
  void abort_run();

  Btn b_mode_, b_start_;
  volatile int click_req_ = 0;
  int click_phase_ = 0;
  uint32_t click_t_ = 0;
  uint32_t t_log_ = 0;
};

}  // namespace mm
