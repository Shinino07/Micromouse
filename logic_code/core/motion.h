#pragma once
#include <cstdint>
#include "moves.h"

namespace mm {

struct TickIn {
  int32_t enc_l = 0, enc_r = 0;
  float   gyro_z = 0;
  bool    gyro_enc = false;
  float   ir[cfg::IR_COUNT] = {};
  float   vbat = cfg::VBAT_NOMINAL;
};
struct TickOut { float duty_l = 0, duty_r = 0; };

struct WallObs {
  bool left, right, front;
  float dl, dr, dfl, dfr;
  float fmin, fmax;
  int fpct;
  int fpct2;
};

class Motion {
 public:
  void init();
  void tick(const TickIn& in, TickOut& out);

  bool push(const Move& m);
  int  free_slots() const;
  bool busy() const { return active_ || head_ != tail_; }
  void abort();
  void enable(bool on);

  bool take_obs(WallObs& o);
  bool take_guard(float& mm, int& turn) {
    if (!guard_hit_) return false;
    guard_hit_ = false;
    mm = guard_mm_;
    turn = guard_turn_;
    return true;
  }
  bool fault() const { return fault_; }
  void clear_fault() { fault_ = false; }

  void start_gyro_cal(uint16_t ms);
  bool gyro_cal_busy() const { return cal_left_ > 0; }

  float ir(int i) const { return ir_f_[i]; }
  float speed() const { return v_meas_; }
  float heading() const { return th_meas_; }
  uint32_t moves_done() const { return moves_done_; }
  float max_pos_err() const { return max_es_; }
  float v_ref() const { return v_ref_; }
  float w_ref() const { return w_ref_; }
  float th_ref() const { return th_ref_; }
  int move_type() const { return active_ ? static_cast<int>(mv_.type) : -1; }
  int move_turn() const { return mv_.turn; }
  float move_s() const { return ms_; }
  float pos_err() const { return s_ref_ - s_meas_; }
  float wall_term_dbg() const { return wall_dbg_; }

 private:
  void start_next();
  void step_move();
  void control(const TickIn& in, TickOut& out);
  float wall_error(bool& valid);
  void edge_correction();
  void sense();
  void presample();
  bool front_guard();
  void diag_track();
  void heading_from_wall(float e);

  static constexpr int QN = 128;
  Move q_[QN];
  volatile uint16_t head_ = 0, tail_ = 0;

  Move mv_{};
  bool active_ = false;
  float ms_ = 0;
  float t_move_ = 0;
  float th_start_ = 0;
  float phi_ = 0, w_piv_ = 0;
  bool sensed_ = false;
  bool pre_valid_ = false, pre_l_ = false, pre_r_ = false;
  int fw_n_ = 0, fw_hit_ = 0, fw_hit2_ = 0;
  int guard_n_ = 0;
  volatile bool guard_hit_ = false;
  float guard_mm_ = 0.0f;
  int guard_turn_ = 0;
  float fw_min_ = 300.0f, fw_max_ = 0.0f;
  bool settling_ = false;

  float s_ref_ = 0, v_ref_ = 0, a_ref_ = 0;
  float th_ref_ = 0, w_ref_ = 0, al_ref_ = 0;
  float s_meas_ = 0, v_meas_ = 0, th_meas_ = 0, w_meas_ = 0;
  int32_t enc_l0_ = 0, enc_r0_ = 0;
  bool enc_init_ = false;
  float gyro_bias_ = 0;
  double cal_sum_ = 0;
  uint16_t cal_left_ = 0, cal_n_ = 0;

  float ir_raw_[cfg::IR_COUNT] = {};
  float ir_prev_[cfg::IR_COUNT] = {};
  float ir_f_[cfg::IR_COUNT] = {};
  float blank_until_[2] = {};
  float ir_sf_[2] = {300.0f, 300.0f};
  bool side_present_[2] = {};
  float present_since_[2] = {-1000.0f, -1000.0f};
  int last_edge_k_[2] = {-9, -9};
  int last_rise_k_[2] = {-9, -9};
  bool wall_active_ = false;
  bool front_align_ = false;
  float fa_ang_ = 0, fa_dist_ = 0;

  volatile bool obs_ready_ = false;
  WallObs obs_{};

  bool enabled_ = false;
  volatile bool fault_ = false;
  uint16_t fault_cnt_ = 0;
  uint32_t moves_done_ = 0;
  float max_es_ = 0;
  float wall_dbg_ = 0;
  float i_pos_ = 0, i_ang_ = 0;
  float vbat_ = cfg::VBAT_NOMINAL;
  float dmin_[2] = {}, dval_[2] = {}, ds_[2] = {};
  bool dvalid_[2] = {};
  bool darmed_[2] = {};
  static constexpr int WH_N = 16;
  float wh_s_[WH_N] = {}, wh_e_[WH_N] = {};
  int wh_n_ = 0;
  float lat_est_ = 0;
  bool lat_valid_ = false;
  bool diag_active_ = false;
 public:
  float dbg_e_ = 0; int dbg_side_ = 0; bool dbg_new_ = false;
 private:
};

}
