#pragma once
#include <cstdint>
#include "motion.h"

namespace mm {

enum class Mode : uint8_t {
  AUTO,
  FOLLOW,
  EXPLORE,
  FAST_SAFE,
  FAST_MAX,
  TEST_STRAIGHT,
  TEST_PIVOT,
  TEST_SENSORS,
  ERASE,
  COUNT
};

class Mouse {
 public:
  void init();
  void loop();
  void tick(const TickIn& in, TickOut& out);

  void start(Mode m);
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
  void go();
  void begin_run(Mode m);
  void end_run();
  void handle_fault();

  void explore_begin(Phase ph, bool from_back);
  void explore_update();
  void explore_push(Dir d);
  void follow_look();
  bool explore_target(CellMask& tgt);
  bool explore_decide(Dir& d, bool at_center, float creep);
  void center_push(Dir d, bool front, float creep);
  bool cell_known(int x, int y) const;
  void finish_home(bool at_center = false, float creep = 0.0f);

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
  bool follow_ = false;
  enum class Look : uint8_t { NONE, WAIT_STOP, SAMPLE };
  Look look_ = Look::NONE;
  bool look_retry_ = false;
  float look_adv_ = 0.0f;
  uint32_t look_t_ = 0, look_ms_ = 0;
  int look_n_ = 0, look_f_ = 0, look_l_ = 0, look_r_ = 0;
  float look_sum_[cfg::IR_COUNT] = {};
  bool obs_front_ = false;
  bool armed_cal_ = false;
  bool at_center_ = false;
  uint32_t t_run_ = 0, t_state_ = 0;
  uint32_t goal_ms_[4] = {};
  int runs_done_ = 0;
  bool fault_seen_ = false;

  static constexpr int MAXM = 200;
  Move moves_[MAXM];
  int moves_n_ = 0, moves_fed_ = 0;
  Dir end_head_ = NORTH;
  uint8_t end_cell_ = 0;

  struct Btn {
    bool raw = false, down = false;
    bool pressed = false, released = false;
    bool skip_release = false;
    bool long_fb = false;
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

}
