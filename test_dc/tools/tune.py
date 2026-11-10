"""Chinh dong co + PID cho micromouse qua ST-Link (firmware test_dc). Khong can bam nut.

  python tune.py dir                 nhac robot len (banh khong cham gi): tim chieu motor, toc do khong tai
  python tune.py straight [mm] [v]   robot tu chay thang CHAM (mac dinh 400 mm, 150 mm/s)
  python tune.py cpm <mm_that>       sua xung/mm theo quang duong do bang thuoc cua lan straight vua roi
  python tune.py ident               chay toi-lui nhieu nac toc do + xoay tai cho -> do KS, KV, KA, ROT_INERTIA
  python tune.py verify [wn] [wn_r]  chay thu PID day du (bang thong rad/s), in sai so + ve do thi
  python tune.py sweep               thu nhieu bang thong, chon bo tot nhat, in dong cho config.h

Bam tuong trong hanh lang me cung (rut ST-Link ra chay bang pin):
  python tune.py walls               doc 5 mat IR (mm) dang thay
  python tune.py walls center        robot dang dat GIUA hanh lang -> luu khoang cach chuan 2 mat ben
  python tune.py arm wall [mm] [v]   nap san lenh chay thang co bam tuong (mac dinh 540 mm, 300 mm/s)
  python tune.py arm straight [mm] [v]   nap san lenh chay thang KHONG bam tuong (de so sanh)
     -> rut ST-Link, dat robot vao hanh lang, bam-nha SW2: 1 s sau robot chay. Bam lai de chay lai.
  python tune.py runs                cam lai ST-Link: doc ket qua tung lan + do thi lan cuoi

Ket qua luu o tools/tune_state.json, do thi o tools/output/.
Bat ky luc nao: bam SW2 hoac SW3 la motor tat ngay.
"""
import json
import os
import re
import struct
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ELF = os.path.join(os.path.dirname(HERE), "build", "Debug", "test_dc.elf")
STATE = os.path.join(HERE, "tune_state.json")
OUT = os.path.join(HERE, "output")
PLUG = r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
CLI = PLUG + r"\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
NM = PLUG + r"\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-nm.exe"

# PHAI KHOP voi enum trong Core/Inc/dc_test.h
PARAMS = ["DUTY_L", "DUTY_R", "TIME_MS", "MSIGN_L", "MSIGN_R", "CPM",
          "NSTEP", "V0", "V1", "V2", "V3", "H0", "H1", "H2", "H3", "ACC",
          "ROT", "ROT_V", "ROT_ACC",
          "KS", "KV", "KA", "KP", "KD", "KI",
          "KV_R", "KA_R", "KP_R", "KD_R", "KI_R",
          "DUTY_MAX", "ERR_MAX",
          "WALL_BW", "WALL_KMAX", "WALL_LIM", "REF_L", "REF_R", "WALL_TH", "HALF_TRACK", "FRONT_STOP",
          "WALL_HEAD", "DECAY",
          "RNSTEP", "RV0", "RV1", "RV2", "RV3", "RH0", "RH1", "RH2", "RH3", "RACC", "LOG_DIV", "BIAS_MS",
          "GYRO_ROT", "GSIGN"]
LOG_WORDS = 8         # sizeof(DcLog) / 4
IR_NAMES = ["S1 ben phai", "S2 cheo phai", "S3 truoc", "S4 cheo trai", "S5 ben trai"]
CMD_OPEN, CMD_PROFILE = 1, 2
STATUS = {2: "xong", 3: "DUNG vi bam nut", 4: "DUNG vi sai so bam qua lon", 5: "DUNG vi het gio",
          6: "DUNG vi gap tuong truoc", 9: "do gyro xoay tay", 11: "HUY: GYRO KHONG CHAY"}
# bam tuong: giong logic_code/core/config.h
WALL = {"WALL_BW": 4.0, "WALL_KMAX": 0.015, "WALL_LIM": 0.12, "HALF_TRACK": 36.0, "WALL_HEAD": 3.0,
        "FRONT_STOP": 60.0}
ENC_SIGN_L, ENC_SIGN_R = +1, -1
DT = 0.002            # log moi 2 ms
VBAT_NOMINAL = 7.6    # = config.h: he so volt = he so duty * VBAT_NOMINAL (port chua do pin)
ZETA = 0.8            # = config.h
DUTY_MAX = 0.6

DEFAULT = {"cpm": 3.038, "msign_l": 1, "msign_r": 1, "dir_ok": False,
           "ks": 0.04, "kv": 0.0010, "ka": 0.00005, "kv_r": 0.0010, "ka_r": 0.00007,
           "ident_ok": False, "wn": 10.0, "wn_r": 15.0}


# ---------------------------------------------------------------- ST-Link
def load_symbols():
    out = subprocess.run([NM, ELF], capture_output=True, text=True).stdout
    return {l.split()[-1]: int(l.split()[0], 16) for l in out.splitlines() if len(l.split()) == 3}


SYM = load_symbols()


SWD_FREQ = "100"   # kHz: day SWD noi dai -> 480 kHz da doc sai 1 byte, 100 kHz on dinh


def cli(args, timeout=120, verify_ok=False):
    r = subprocess.run([CLI, "-c", "port=SWD", "mode=HOTPLUG", "freq=" + SWD_FREQ] + args, capture_output=True,
                       text=True, timeout=timeout)
    if verify_ok and "Failed to download data" in r.stdout and "Unable to get core" not in r.stdout:
        return r.stdout   # ghi g_cmd: firmware nhan lenh xoa ngay ve 0 nen CLI doc lai thay khac -> binh thuong
    if re.search(r"Error[: ]", r.stdout):
        sys.exit("Loi ST-Link (kiem tra day SWD, CubeIDE khong duoc dang debug):\n" + r.stdout[-500:])
    return r.stdout


def read_words(addr, n):
    args = []
    CH = 4096   # moi lan -r32 toi da 32 KB -> chia 16 KB
    for i in range(0, n, CH):
        args += ["-r32", hex(addr + 4 * i), str(4 * min(CH, n - i))]
    out = cli(args)
    words = {}
    for m in re.finditer(r"0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{8}[ \t]*)+)", out):
        base = int(m.group(1), 16)
        for k, w in enumerate(m.group(2).split()):
            words[base + 4 * k] = int(w, 16)
    try:
        return [words[addr + 4 * i] for i in range(n)]
    except KeyError:
        sys.exit("Doc thieu du lieu tu robot:\n" + out[-500:])


def read_vars(names):
    args = []
    for n in names:
        args += ["-r32", hex(SYM[n]), "4"]
    out = cli(args)
    vals = {}
    for n in names:
        m = re.search("0x%08X" % SYM[n] + r"\s*:\s*([0-9A-Fa-f]{8})", out, re.I)
        if not m:
            sys.exit("Khong doc duoc %s:\n%s" % (n, out[-500:]))
        vals[n] = int(m.group(1), 16)
    return vals


def f2u(x):
    return struct.unpack("<I", struct.pack("<f", float(x)))[0]


def write_params(prm, arm=0):
    prm = dict(prm)
    prm.setdefault("DUTY_MAX", DUTY_MAX)
    args = ["-w32", hex(SYM["g_arm"]), "0x%08X" % arm]
    for i, name in enumerate(PARAMS):
        args += ["-w32", hex(SYM["g_p"] + 4 * i), "0x%08X" % f2u(prm.get(name, 0.0))]
    cli(args)
    return prm


def read_log(n):
    """doc log, doc lai neu nhieu SWD lam hong so lieu"""
    for _ in range(3):
        if n <= 4:
            return None
        log = parse_log(read_words(SYM["g_log"], LOG_WORDS * n))
        jump = max(np.max(np.abs(np.diff(log["enc_l"]))), np.max(np.abs(np.diff(log["enc_r"]))))
        bad = jump > 30 or np.max(np.abs(log["raw_l"])) > 1.0 or np.max(np.abs(log["raw_r"])) > 1.0 \
            or not np.all(np.isfinite(log["s_ref"])) or not np.all(np.isfinite(log["d_ref"]))
        if not bad:
            return log
        print("  (log doc ve bi nhieu, doc lai)")
    return None


def run_cmd(cmd, prm, label):
    """Ghi tham so, ra lenh, doi xong, tra ve log da giai ma."""
    seq0 = read_vars(["g_seq"])["g_seq"]
    prm = write_params(prm)
    cli(["-w32", hex(SYM["g_cmd"]), "0x%08X" % cmd], verify_ok=True)
    t_end = time.time() + prm["TIME_MS"] / 1000.0 + 20
    while True:
        time.sleep(0.4)
        v = read_vars(["g_seq", "g_status", "g_log_n"])
        if v["g_seq"] != seq0:
            break
        if time.time() > t_end:
            sys.exit("Robot khong tra loi (firmware test_dc da nap chua?)")
    st = v["g_status"]
    n = v["g_log_n"]
    print("  [%s] %s, %d mau log" % (label, STATUS.get(st, "trang thai %d" % st), n))
    return st, read_log(n)


def i16(col):
    return col.astype(np.uint16).view(np.int16).astype(float)


def parse_log(words):
    a = np.array(words, dtype=np.uint32).reshape(-1, LOG_WORDS)
    return {"enc_l": a[:, 0].copy().view(np.int32).astype(float),
            "enc_r": a[:, 1].copy().view(np.int32).astype(float),
            "raw_l": i16(a[:, 2] & 0xFFFF) / 10000.0, "raw_r": i16(a[:, 2] >> 16) / 10000.0,
            "s_ref": a[:, 3].copy().view(np.float32).astype(float),
            "d_ref": a[:, 4].copy().view(np.float32).astype(float),
            "ir_l": i16(a[:, 5] & 0xFFFF) / 10.0, "ir_r": i16(a[:, 5] >> 16) / 10.0,
            "ir_f": i16(a[:, 6] & 0xFFFF) / 10.0, "wall": i16(a[:, 6] >> 16) / 100.0,
            "gyro_deg": a[:, 7].copy().view(np.float32).astype(float),
            "t": np.arange(len(a)) * DT}


# ---------------------------------------------------------------- xu ly so lieu
def deriv(x, k=None):
    """dao ham trung tam qua 2k mau (40 ms): encoder chi 0.33 mm/xung nen can lay rong"""
    k = k or max(3, int(round(0.02 / DT)))
    y = np.full(len(x), np.nan)
    if len(x) > 2 * k:
        y[k:-k] = (x[2 * k:] - x[:-2 * k]) / (2 * k * DT)
    return y


def states(log, st):
    """vi tri tung banh (mm) va duty theo chieu tien (bo dau motor)"""
    sl = ENC_SIGN_L * (log["enc_l"] - log["enc_l"][0]) / st["cpm"]
    sr = ENC_SIGN_R * (log["enc_r"] - log["enc_r"][0]) / st["cpm"]
    ul = log["raw_l"] / st["msign_l"]
    ur = log["raw_r"] / st["msign_r"]
    return sl, sr, ul, ur


def window_rows(x, u, sat, W=None, step=None, vmin=40.0):
    """Mo hinh u = KS*sgn(v) + KV*v + KA*a, viet dang tich phan tren cua so W mau (50 ms):
       mean(u) = KS*mean(sgn v) + KV*(x1-x0)/T + KA*(v1-v0)/T  -> khong phai dao ham 2 lan so lieu thieu min"""
    W = W or int(round(0.05 / DT))
    step = step or max(1, int(round(0.006 / DT)))
    v = deriv(x)
    rows, ys = [], []
    T = W * DT
    for i in range(0, len(x) - W, step):
        j = i + W
        seg = v[i:j + 1]
        if np.any(np.isnan(seg)) or np.min(np.abs(seg)) < vmin or np.any(sat[i:j]):
            continue
        if np.sign(seg[0]) != np.sign(seg[-1]):
            continue
        rows.append([np.mean(np.sign(seg)), (x[j] - x[i]) / T, (v[j] - v[i]) / T])
        ys.append(np.mean(u[i:j]))
    return rows, ys


def fit(rows, ys, name):
    A = np.array(rows)
    y = np.array(ys)
    if len(y) < 15:
        sys.exit("Qua it so lieu de do %s (%d cua so) - robot co chay khong?" % (name, len(y)))
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    r2 = 1 - np.sum((y - pred) ** 2) / np.sum((y - np.mean(y)) ** 2)
    print("  %s: KS = %.4f  KV = %.6f duty/(mm/s)  KA = %.7f duty/(mm/s^2)   R2 = %.3f  (%d cua so)"
          % (name, coef[0], coef[1], coef[2], r2, len(y)))
    return coef, r2


def gains(st, wn, wn_r):
    """cung cong thuc voi logic_code/core/config.h (o day don vi duty)"""
    kp = st["ka"] * wn ** 2
    kd = max(0.0, 2 * ZETA * wn * st["ka"] - st["kv"])
    kp_r = st["ka_r"] * wn_r ** 2
    kd_r = max(0.0, 2 * ZETA * wn_r * st["ka_r"] - st["kv_r"])
    return {"KS": st["ks"], "KV": st["kv"], "KA": st["ka"], "KP": kp, "KD": kd, "KI": 2 * kp,
            "KV_R": st["kv_r"], "KA_R": st["ka_r"], "KP_R": kp_r, "KD_R": kd_r, "KI_R": 4 * kp_r}


def base_prm(st):
    return {"MSIGN_L": st["msign_l"], "MSIGN_R": st["msign_r"], "CPM": st["cpm"], "DECAY": st.get("decay", 1)}


def trans_prm(st, steps, acc, wn, wn_r, err=25.0):
    prm = base_prm(st)
    prm.update(gains(st, wn, wn_r))
    prm.update({"NSTEP": len(steps), "ACC": acc, "TIME_MS": 9000, "ERR_MAX": err})
    for i, (v, h) in enumerate(steps):
        prm["V%d" % i] = v
        prm["H%d" % i] = h
    return prm


def rot_prm(st, dist, v, acc, wn, wn_r, err=25.0):
    prm = base_prm(st)
    prm.update(gains(st, wn, wn_r))
    prm.update({"ROT": dist, "ROT_V": v, "ROT_ACC": acc, "TIME_MS": 6000, "ERR_MAX": err})
    return prm


def metrics(log, st):
    sl, sr, ul, ur = states(log, st)
    s, d = 0.5 * (sl + sr), 0.5 * (sr - sl)
    e, ed = log["s_ref"] - s, log["d_ref"] - d
    ref = log["s_ref"] if np.ptp(log["s_ref"]) > np.ptp(log["d_ref"]) else log["d_ref"]
    pos = s if ref is log["s_ref"] else d
    moving = np.abs(np.diff(ref, prepend=ref[0])) > 1e-4
    end = np.nonzero(moving)[0][-1] if np.any(moving) else len(ref) - 1
    direction = np.sign(ref[end] - ref[0]) or 1.0
    over = max(0.0, np.max(direction * (pos[end:] - ref[end]))) if end < len(ref) else 0.0
    u = 0.5 * (ul + ur)
    k = 5
    smooth = np.convolve(u, np.ones(2 * k + 1) / (2 * k + 1), mode="same")
    chatter = float(np.std((u - smooth)[k:-k])) if len(u) > 4 * k else 0.0
    return {"max_e": float(np.max(np.abs(e))), "rms_e": float(np.sqrt(np.mean(e ** 2))),
            "max_ed": float(np.max(np.abs(ed))), "final": float(abs(ref[-1] - pos[-1])),
            "over": float(over), "chatter": chatter, "dist": float(s[-1]), "rot": float(d[-1])}


def plot(log, st, name, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    os.makedirs(OUT, exist_ok=True)
    sl, sr, ul, ur = states(log, st)
    s, d = 0.5 * (sl + sr), 0.5 * (sr - sl)
    t = log["t"]
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(t, log["s_ref"], "k--", label="s tham chieu")
    ax[0].plot(t, s, label="s do (TB 2 banh)")
    ax[0].plot(t, log["d_ref"], "k:", label="d tham chieu (xoay)")
    ax[0].plot(t, d, label="d do")
    ax[0].set_ylabel("mm")
    ax[0].legend(fontsize=8)
    ax[1].plot(t, log["s_ref"] - s, label="sai so tinh tien")
    ax[1].plot(t, log["d_ref"] - d, label="sai so xoay (mm/banh)")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("mm")
    ax[1].legend(fontsize=8)
    ax[2].plot(t, ul, label="duty trai")
    ax[2].plot(t, ur, label="duty phai")
    ax[2].set_ylabel("duty")
    ax[2].set_xlabel("s")
    ax[2].legend(fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name + ".png"), dpi=110)
    plt.close(fig)


def load_state():
    st = dict(DEFAULT)
    if os.path.exists(STATE):
        st.update(json.load(open(STATE, encoding="utf-8")))
    return st


def save_state(st):
    json.dump(st, open(STATE, "w", encoding="utf-8"), indent=2)


def need(st, key, msg):
    if not st.get(key):
        sys.exit(msg)


# ---------------------------------------------------------------- cac buoc
def cmd_dir(st):
    print("Nhac robot len, 2 banh quay tu do. Moi banh se quay toi ~0.6 s o 2 muc duty.")
    res = {}
    for side in ("L", "R"):
        speeds = []
        for duty in (0.15, 0.30):
            prm = {"DUTY_L": duty if side == "L" else 0.0, "DUTY_R": duty if side == "R" else 0.0,
                   "TIME_MS": 700, "DUTY_MAX": 0.35, "DECAY": st.get("decay", 1)}
            stt, log = run_cmd(CMD_OPEN, prm, "banh %s duty +%.2f" % ("trai" if side == "L" else "phai", duty))
            if stt != 2 or log is None:
                sys.exit("Dung giua chung - lam lai")
            sl = ENC_SIGN_L * (log["enc_l"] - log["enc_l"][0]) / st["cpm"]
            sr = ENC_SIGN_R * (log["enc_r"] - log["enc_r"][0]) / st["cpm"]
            own, other = (sl, sr) if side == "L" else (sr, sl)
            tail = slice(int(0.45 / DT), len(own))
            v = np.polyfit(log["t"][tail], own[tail], 1)[0]   # mm/s banh (theo chieu tien cua encoder)
            print("     banh dang thu: %+7.1f mm/s, banh kia nhuc nhich %.1f mm" % (v, np.ptp(other)))
            if np.ptp(other) > 3 and abs(own[-1]) < 3:
                sys.exit("!! Lenh cho banh %s lai lam quay banh KIA -> day motor trai/phai bi doi cho." % side)
            speeds.append(v)
        v1, v2 = speeds
        if abs(v2) < 20:
            sys.exit("Banh %s khong quay o duty 0.30: kiem tra pin, day motor, driver." % side)
        sign = 1 if v2 > 0 else -1
        if abs(v1) > 20:   # 2 diem tren duong thang duty = KS + KV*v
            kv = 0.15 / (abs(v2) - abs(v1))
            ks = max(0.0, 0.15 - kv * abs(v1))
        else:              # duty 0.15 chua thang duoc ma sat
            kv = 0.15 / abs(v2)
            ks = 0.15
        res[side] = (sign, kv, ks, abs(v2))
        print("  => MOTOR_SIGN_%s = %+d   (khong tai: KV ~ %.5f duty/(mm/s), KS ~ %.3f, %.0f mm/s @ 0.30)"
              % (side, sign, kv, ks, abs(v2)))
    st["msign_l"], st["msign_r"] = res["L"][0], res["R"][0]
    if not st["ident_ok"]:   # uoc luong tam cho lan chay tren ban dau tien
        st["kv"] = st["kv_r"] = 0.5 * (res["L"][1] + res["R"][1])
        st["ks"] = max(0.04, 0.5 * (res["L"][2] + res["R"][2]) * 1.5)
        st["ka"] = st["kv"] * 0.05
        st["ka_r"] = st["ka"] * 1.4
    st["dir_ok"] = True
    save_state(st)
    print("\nDa luu. Toc do khong tai lech nhau %.1f%% giua 2 banh."
          % (100 * abs(res["L"][3] - res["R"][3]) / max(res["L"][3], res["R"][3])))


def cmd_straight(st, dist=400.0, v=150.0):
    need(st, "dir_ok", "Chay 'python tune.py dir' truoc.")
    print("Robot se tu chay thang %.0f mm o %.0f mm/s." % (dist, v))
    acc = 600.0
    hold = max(0.0, dist - v * v / acc)
    # chua do mo hinh: dung bang thong 15 (an toan ca khi KA uoc luong lech 3-5 lan vi KV giu tat dan)
    wn, wn_r = (st["wn"], st["wn_r"]) if st["ident_ok"] else (15.0, 15.0)
    prm = trans_prm(st, [(v, hold)], acc, wn, wn_r, err=30.0)
    stt, log = run_cmd(CMD_PROFILE, prm, "chay thang")
    if log is None:
        return
    m = metrics(log, st)
    sl, sr, _, _ = states(log, st)
    st["last_straight_mm"] = m["dist"]
    save_state(st)
    plot(log, st, "straight", "Chay thang %.0f mm @ %.0f mm/s" % (dist, v))
    print("  Encoder: robot da di %.1f mm (trai %.1f, phai %.1f), sai so bam lon nhat %.1f mm"
          % (m["dist"], sl[-1], sr[-1], m["max_e"]))
    if stt == 2:
        print("  => Do quang duong THAT bang thuoc roi chay: python tune.py cpm <so_mm_do_duoc>")


def cmd_cpm(st, real):
    if "last_straight_mm" not in st:
        sys.exit("Chua co lan straight nao.")
    old = st["cpm"]
    st["cpm"] = old * st["last_straight_mm"] / real
    save_state(st)
    print("xung/mm: %.4f -> %.4f  (sai %.2f%%)" % (old, st["cpm"], 100 * (st["cpm"] / old - 1)))


def read_ir():
    words = read_words(SYM["g_ir_mm"], 5)
    return [struct.unpack("<f", struct.pack("<I", w))[0] for w in words]


def cmd_walls(st, center=False):
    acc = np.zeros(5)
    N = 5
    for _ in range(N):
        acc += np.minimum(read_ir(), 999.0)
        time.sleep(0.1)
    mm = acc / N
    for name, d in zip(IR_NAMES, mm):
        print("  %-13s %s" % (name, "khong thay" if d >= 990 else "%6.1f mm" % d))
    if center:
        if mm[4] >= 200 or mm[0] >= 200:
            sys.exit("Phai thay ca 2 tuong ben (dat robot giua hanh lang co 2 tuong).")
        st["ref_l"], st["ref_r"] = float(mm[4]), float(mm[0])
        # robot sat 1 ben thi tuong ben kia cach ~ chuan + 50 mm -> nguong chuan + 70 (o ben khong co tuong: > 230 mm)
        st["wall_th"] = float(min(130.0, max(mm[4], mm[0]) + 70.0))
        save_state(st)
        print("Da luu: chuan trai %.1f mm, chuan phai %.1f mm, nguong co tuong %.0f mm"
              % (st["ref_l"], st["ref_r"], st["wall_th"]))


def wall_prm(st, on):
    prm = dict(WALL)
    if not on:
        prm["WALL_BW"] = 0.0
    prm.update({"REF_L": st.get("ref_l", 0.0), "REF_R": st.get("ref_r", 0.0), "WALL_TH": st.get("wall_th", 0.0)})
    return prm


def save_flash(arm):
    """ghi g_p + g_arm vao flash robot (chiu duoc reset khi rut/cam day)"""
    cli(["-w32", hex(SYM["g_arm"]), "0x%08X" % arm])
    seq0 = read_vars(["g_seq"])["g_seq"]
    cli(["-w32", hex(SYM["g_cmd"]), "0x%08X" % 3], verify_ok=True)
    t_end = time.time() + 15
    while time.time() < t_end:
        time.sleep(0.5)
        v = read_vars(["g_seq", "g_status", "g_arm"])
        if v["g_seq"] != seq0:
            if v["g_status"] != 7:
                sys.exit("Ghi flash THAT BAI (trang thai %d)" % v["g_status"])
            return
    sys.exit("Robot khong tra loi lenh ghi flash")


IDENT_STEPS = [(150.0, 80.0), (300.0, 100.0), (-150.0, 80.0), (-300.0, 100.0)]   # toi roi lui ve cho cu


def ident_prm(st):
    """1 lan bam: tinh tien toi-lui 4 nac, roi xoay trai-phai 4 nac (ve lai huong cu), log 4 ms"""
    prm = trans_prm(st, IDENT_STEPS, 1500.0, 20.0, 20.0, err=60.0)
    prm.update({"RNSTEP": len(IDENT_STEPS), "RACC": 1500.0, "LOG_DIV": 4, "TIME_MS": 12000, "FRONT_STOP": 60.0})
    for i, (v, h) in enumerate(IDENT_STEPS):
        prm["RV%d" % i] = v
        prm["RH%d" % i] = h
    return prm


def cmd_arm(st, kind="wall", dist=540.0, v=300.0):
    if kind == "gyro":
        cmd_arm_gyro(st)
        return
    if kind == "gyro10":
        cmd_arm_gyro10(st)
        return
    if kind == "ident":
        need(st, "dir_ok", "Chay 'python tune.py dir' truoc.")
        write_params(ident_prm(st), arm=1)
        save_flash(1)
        st["arm_kind"] = "ident"
        save_state(st)
        print("Da nap bai DO MO HINH (1 lan bam). Rut ST-Link, dat robot tren ban (truoc mat trong ~25 cm),")
        print("bam-nha SW2: robot chay toi ~24 cm, lui ve, roi xoay qua lai (~6 s). Cho 2 s roi cam day,")
        print("chay: python tune.py ident flash")
        return
    if kind == "off":
        save_flash(0)
        print("Da huy hen gio (ca trong flash).")
        return
    need(st, "dir_ok", "Chay 'python tune.py dir' truoc.")
    if kind == "wall":
        need(st, "ref_l", "Dat robot giua hanh lang roi chay 'python tune.py walls center' truoc.")
    acc = 1500.0
    hold = max(0.0, dist - v * v / acc)
    wn, wn_r = (st["wn"], st["wn_r"]) if st["ident_ok"] else (20.0, 20.0)
    prm = trans_prm(st, [(v, hold)], acc, wn, wn_r, err=30.0 if st["ident_ok"] else 60.0)
    prm.update(wall_prm(st, kind == "wall"))
    # dung truoc tuong: quang phanh v^2/(2*4000) + tre do/bam ~10 mm + chua 25 mm
    prm["FRONT_STOP"] = max(60.0, v * v / 8000.0 + 35.0)
    write_params(prm, arm=1)
    save_flash(1)
    st["arm_run0"] = 0   # robot co the reset khi rut day -> dem lan chay tu 0
    st["arm_kind"] = kind
    save_state(st)
    print("Da nap: chay thang %.0f mm @ %.0f mm/s, %s." % (dist, v, "CO bam tuong" if kind == "wall" else "KHONG bam tuong"))
    print("Rut ST-Link -> dat robot vao hanh lang -> bam roi nha SW2 -> 1 s sau robot chay.")
    print("Muon chay lai: dat lai robot, bam-nha SW2 lan nua. Xong: cam ST-Link, chay 'python tune.py runs'.")


def lateral(log, st):
    """lech ngang (mm, + = robot lech phai) tinh lai tu 2 mat ben, giong firmware"""
    dl, dr = log["ir_l"], log["ir_r"]
    th = st.get("wall_th", 0.0)
    e = np.full(len(dl), np.nan)
    okl = (dl < th) & (dl >= 41.0)   # giong firmware: mat bao hoa thi bo
    okr = (dr < th) & (dr >= 31.0)
    both = okl & okr
    e[both] = 0.5 * ((dl - st["ref_l"]) - (dr - st["ref_r"]))[both]
    only_l = okl & ~both
    e[only_l] = (dl - st["ref_l"])[only_l]
    only_r = okr & ~both
    e[only_r] = (st["ref_r"] - dr)[only_r]
    return e


def plot_wall(log, st, name, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    os.makedirs(OUT, exist_ok=True)
    sl, sr, ul, ur = states(log, st)
    s = 0.5 * (sl + sr)
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    ax[0].plot(s, np.where(log["ir_l"] < 300, log["ir_l"], np.nan), label="mat ben trai (mm)")
    ax[0].plot(s, np.where(log["ir_r"] < 300, log["ir_r"], np.nan), label="mat ben phai (mm)")
    ax[0].axhline(st.get("ref_l", 0), color="C0", ls=":")
    ax[0].axhline(st.get("ref_r", 0), color="C1", ls=":")
    ax[0].set_ylabel("mm")
    ax[0].legend(fontsize=8)
    ax[1].plot(s, lateral(log, st), label="lech ngang (+ = lech phai)")
    ax[1].plot(s, log["wall"], label="bu huong bam tuong (mm tren banh)")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("mm")
    ax[1].legend(fontsize=8)
    ax[2].plot(s, ul, label="duty trai")
    ax[2].plot(s, ur, label="duty phai")
    ax[2].set_xlabel("quang duong (mm)")
    ax[2].legend(fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name + ".png"), dpi=110)
    plt.close(fig)


RUNLOG_ADDR, RUNLOG_MAGIC, RUNS_MAX = 0x080C0000, 0x44434C31, 16
RUNLOG_HDR = 2 + RUNS_MAX * 4 + 1


def cmd_runs(st):
    # robot ghi ket qua vao flash sau moi lan chay (cam/rut ST-Link lam robot reset, RAM mat)
    hdr = read_words(RUNLOG_ADDR, RUNLOG_HDR)
    if hdr[0] != RUNLOG_MAGIC or hdr[1] == 0 or hdr[1] > RUNS_MAX:
        print("Chua co lan chay nao duoc luu (robot co chay khong? sau khi chay xong cho ~2 s robot moi ghi xong).")
        return
    n, first = hdr[1], 0
    w = hdr[2:2 + 4 * RUNS_MAX]
    print("Lan  ket qua                      trai    phai   (mm)  phai-trai  thoi gian")
    for i in range(first, n):
        status, dl, dr, ms = w[4 * i:4 * i + 4]
        dl = struct.unpack("<i", struct.pack("<I", dl))[0] * ENC_SIGN_L / st["cpm"]
        dr = struct.unpack("<i", struct.pack("<I", dr))[0] * ENC_SIGN_R / st["cpm"]
        print("%3d  %-26s %7.1f %7.1f        %+6.1f   %5.2f s" % (i - first + 1, STATUS.get(status, str(status)),
                                                              dl, dr, dr - dl, ms / 1000.0))
    ln = hdr[RUNLOG_HDR - 1]
    if not 4 < ln <= 2000:
        return
    log = parse_log(read_words(RUNLOG_ADDR + 4 * RUNLOG_HDR, LOG_WORDS * ln))
    e = lateral(log, st)
    sl, sr, _, _ = states(log, st)
    s = 0.5 * (sl + sr)
    ok = ~np.isnan(e)
    if np.any(ok):
        tail = ok & (s > s[-1] - 100)
        print("Lan cuoi: lech ngang luc dau %+.1f mm, lon nhat %.1f mm, 100 mm cuoi trung binh %+.1f mm"
              % (e[ok][0], np.max(np.abs(e[ok])), np.mean(e[tail]) if np.any(tail) else float("nan")))
    plot_wall(log, st, "hanh_lang", "Lan chay cuoi trong hanh lang (%s)" % st.get("arm_kind", ""))
    print("Do thi: tools/output/hanh_lang.png")


def fit_ident_log(log, st):
    """tach phan tinh tien (d_ref = 0) va phan xoay (sau do), hoi quy tung phan"""
    sl, sr, ul, ur = states(log, st)
    sat = np.maximum(np.abs(ul), np.abs(ur)) > DUTY_MAX * 0.97
    rot = np.nonzero(np.abs(log["d_ref"]) > 1e-3)[0]
    k0 = rot[0] if len(rot) else len(sl)
    T_rows, T_y = window_rows((0.5 * (sl + sr))[:k0], (0.5 * (ul + ur))[:k0], sat[:k0])
    R_rows, R_y = window_rows((0.5 * (sr - sl))[k0:], (0.5 * (ur - ul))[k0:], sat[k0:])
    (ks, kv, ka), r2 = fit(T_rows, T_y, "tinh tien")
    (ks_r, kv_r, ka_r), r2r = fit(R_rows, R_y, "xoay     ")
    return ks, kv, ka, ks_r, kv_r, ka_r, r2, r2r


def cmd_ident_flash(st, binfile=None):
    global DT
    DT = 0.004   # bai do 1 lan bam ghi log moi 4 ms
    if binfile:   # thu tren may tinh (log tu harness)
        log = parse_log(list(np.fromfile(binfile, dtype=np.uint32)))
    else:
        hdr = read_words(RUNLOG_ADDR, RUNLOG_HDR)
        if hdr[0] != RUNLOG_MAGIC or hdr[1] == 0:
            sys.exit("Robot chua luu lan chay nao (da bam SW2 chua? cho 2 s sau khi robot dung roi moi cam day).")
        status = read_words(RUNLOG_ADDR + 8 + 16 * (hdr[1] - 1), 1)[0]
        print("Lan chay cuoi: %s" % STATUS.get(status, str(status)))
        ln = hdr[RUNLOG_HDR - 1]
        log = parse_log(read_words(RUNLOG_ADDR + 4 * RUNLOG_HDR, LOG_WORDS * ln))
    plot(log, st, "ident_1lan", "Do mo hinh 1 lan bam (tinh tien roi xoay)")
    ks, kv, ka, ks_r, kv_r, ka_r, r2, r2r = fit_ident_log(log, st)
    if kv <= 0 or ka <= 0 or ka_r <= 0 or r2 < 0.8:
        sys.exit("Ket qua khong hop ly - xem do thi tools/output/ident_1lan.png")
    if binfile:
        return ks, kv, ka, ks_r, kv_r, ka_r
    st.update({"ks": float(max(ks, 0.0)), "kv": float(kv), "ka": float(ka),
               "kv_r": float(kv_r), "ka_r": float(ka_r), "ks_rot": float(ks_r), "ident_ok": True})
    save_state(st)
    print()
    print("ROT_INERTIA = %.2f ; KV xoay / KV tinh tien = %.2f" % (ka_r / ka, kv_r / kv))
    print_config(st)


EV_ADDR = 0x080A0000
EV_NAMES = {1: "KHOI DONG", 2: "BAT DAU CHAY", 3: "KET THUC", 4: "NAP LENH", 5: "CON SONG", 6: "DO BIAS XONG"}
RST_BITS = {1: "BOR", 2: "PIN/NRST (cam rut day, nut reset)", 3: "POR (MAT NGUON)", 4: "SOFT", 5: "IWDG", 6: "WWDG", 7: "LPWR"}


def cmd_events(st, last=40):
    """doc hop den: moi lan khoi dong / bat dau / ket thuc chay, kem nguyen nhan reset"""
    words = []
    for k in range(8):   # toi da 8 x 4096 tu
        chunk = read_words(EV_ADDR + 16384 * k, 4096)
        words += chunk
        if 0xFFFFFFFF in chunk:
            break
    ev = []
    for i in range(0, len(words) - 1, 2):
        if words[i] == 0xFFFFFFFF:
            break
        ev.append((words[i], words[i + 1]))
    print("Hop den: %d su kien, in %d cai cuoi" % (len(ev), min(last, len(ev))))
    open_run = None
    for w0, ms in ev[-last:]:
        typ, data = (w0 >> 24) & 0xF, w0 & 0xFFFFFF
        txt = EV_NAMES.get(typ, "?%d" % typ)
        if typ == 1:
            flags = [n for b, n in RST_BITS.items() if data >> b & 1]
            extra = "ly do: " + (", ".join(flags) or "khong ro") + ("  | hen gio kieu %d" % (data >> 8 & 0xF) if data >> 8 & 0xF else "")
            if open_run is not None:
                extra += "   <-- !!! RESET GIUA LUC CHAY (lan %d)" % open_run
            open_run = None
        elif typ == 2:
            extra = "lan %d" % ((data >> 4) & 0xFF)
            open_run = (data >> 4) & 0xFF
        elif typ == 3:
            extra = "%s, lan %d, %.2f s" % (STATUS.get(data & 0xF, str(data & 0xF)), (data >> 4) & 0xFF, ((data >> 12) & 0xFFF) / 100.0)
            open_run = None
        elif typ == 5:
            extra = "giay thu %d cua lan chay" % (data & 0xFFFF)
        elif typ == 6:
            extra = "bias %.1f LSB" % ((data & 0xFFFFF) / 10.0)
        else:
            extra = ""
        print("  t=%8.3f s  %-13s %s" % (ms / 1000.0, txt, extra))


GYRO_TURNS_HAND = 10        # so vong xoay tay
GYRO_PIVOT_TURNS = 5        # so vong motor xoay (tinh theo TRACK danh nghia)
TRACK_NOMINAL = 72.0


def gyro_prm(st):
    prm = base_prm(st)
    prm.update(gains(st, st["wn"], st["wn_r"]))
    arc = GYRO_PIVOT_TURNS * np.pi * TRACK_NOMINAL   # quang duong moi banh (mm), + = quay trai
    prm.update({"ROT": arc, "ROT_V": 250.0, "ROT_ACC": 1500.0, "BIAS_MS": 1000, "LOG_DIV": 4,
                "TIME_MS": 120000, "ERR_MAX": 30.0})
    return prm


def cmd_arm_gyro10(st):
    """SW3: robot tu xoay trai DUNG 10 vong theo gyro (dau da do) -> nguoi xem thua/thieu bao nhieu do"""
    need(st, "gyro_sign", "Chay bai motor xoay (SW3 cua 'arm gyro') truoc de biet dau gyro.")
    half = st.get("gyro_track", 72.0) / 2.0
    prm = base_prm(st)
    prm.update(gains(st, st["wn"], st["wn_r"]))
    prm.update({"ROT": 10 * 2 * np.pi * half, "ROT_V": 300.0, "ROT_ACC": 1500.0, "BIAS_MS": 1000, "LOG_DIV": 6,
                "TIME_MS": 120000, "ERR_MAX": 40.0, "GYRO_ROT": 1, "HALF_TRACK": half,
                # dau x he so hien tai: lan sau do lai (sau khi sua he so) robot phai quay dung 3600 do
                "GSIGN": st["gyro_sign"] * st.get("gyro_scale", 1.0)})
    write_params(prm, arm=2)
    save_flash(2)
    st["arm_kind"] = "gyro10"
    save_state(st)
    print("Da nap: SW3 = robot tu xoay trai 10 vong theo gyro (dau %+d, nua truc %.1f mm)." % (st["gyro_sign"], half))


def cmd_gyro_err(st, err_deg):
    """err_deg: robot dung THUA (+) hay THIEU (-) bao nhieu do so voi huong ban dau sau 10 vong theo gyro"""
    old = st.get("gyro_scale", 1.0)
    true = 3600.0 + err_deg               # goc that robot da quay
    scale = old * true / 3600.0           # gyro (da nhan he so cu) bao 3600 do
    track = st["gyro_track"] * old / scale if "gyro_track" in st else None
    print("Goc that %.1f do / gyro bao 3600 do -> GYRO_SCALE %.4f -> %.4f" % (true, old, scale))
    st["gyro_scale"] = scale
    if track:
        print("TRACK_MM tinh lai theo he so moi: %.2f -> %.2f" % (st["gyro_track"], track))
        st["gyro_track"] = track
    save_state(st)
    print()
    print("---- chep vao logic_code/core/config.h ----")
    print("constexpr float GYRO_SIGN  = %+.1ff;" % st["gyro_sign"])
    print("constexpr float GYRO_SCALE = %.4ff;" % scale)
    if track:
        print("constexpr float TRACK_MM   = %.2ff;" % track)


def cmd_arm_gyro(st):
    need(st, "ident_ok", "Can mo hinh motor truoc (python tune.py ident flash).")
    v = read_vars(["g_imu_who"])
    who = v["g_imu_who"] & 0xFF
    names = {0x12: "ICM-20602", 0x11: "ICM-20600", 0xAC: "ICM-20601", 0xAF: "ICM-20608", 0x98: "ICM-20689",
             0x70: "MPU-6500", 0x71: "MPU-9250", 0x73: "MPU-9255", 0x47: "ICM-42688",
             0x2E: "chip tuong thich MPU-6500 (ban clone ghi ICM-20602, da kiem bang gia toc: Z = -1 g)"}
    print("Gyro: WHO_AM_I = 0x%02X -> %s" % (who, names.get(who, "KHONG NHAN RA")))
    if who not in names:
        sys.exit("Khong doc duoc gyro: kiem tra day SPI (PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS) va nguon 3.3 V cho mach gyro.")
    g = []
    for _ in range(5):
        raw = read_vars(["g_gz"])["g_gz"] & 0xFFFF
        g.append(raw - 65536 if raw >= 32768 else raw)
        time.sleep(0.1)
    print("Gyro Z luc dung yen: %s LSB (16.4 LSB = 1 do/s)" % g)
    write_params(gyro_prm(st), arm=2)
    save_flash(2)
    st["arm_kind"] = "gyro"
    save_state(st)


def gyro_analyze(st, hdr, log):
    n = hdr[1]
    runs = [hdr[2 + 4 * i: 6 + 4 * i] for i in range(n)]
    s32 = lambda x: int(x) - (1 << 32) if int(x) >= 1 << 31 else int(x)
    hand = [r for r in runs if r[0] == 9]
    res = {}
    if hand:
        ang = s32(hand[-1][1]) / 1000.0
        bias = s32(hand[-1][2]) / 1000.0
        res["scale"] = 360.0 * GYRO_TURNS_HAND / abs(ang)
        res["hand_sign"] = 1 if ang > 0 else -1
        print("Xoay tay %d vong: gyro do %.2f do, bias %.1f LSB (%.2f do/s) -> GYRO_SCALE = %.4f"
              % (GYRO_TURNS_HAND, ang, bias, bias / 16.4, res["scale"]))
        if not 0.9 < res["scale"] < 1.1:
            print("  !! he so lech > 10%: co the xoay sai so vong (phai dung %d vong) - lam lai bai xoay tay" % GYRO_TURNS_HAND)
    else:
        print("Chua co bai xoay tay (SW2).")
    if runs and runs[-1][0] == 2 and log is not None and np.ptp(log["d_ref"]) > 100:
        sl, sr, _, _ = states(log, st)
        d = 0.5 * (sr - sl)
        dd = d[-1] - d[0]
        gang = log["gyro_deg"][-1]
        sign = 1 if (gang > 0) == (dd > 0) else -1
        scale = res.get("scale", 1.0)
        th = abs(gang) * scale * np.pi / 180.0
        track = 2.0 * abs(dd) / th
        res.update({"sign": sign, "track": track})
        print("Motor xoay trai: banh di %.1f mm, gyro do %.1f do -> GYRO_SIGN = %+d, goc that %.1f do -> TRACK_MM = %.2f"
              % (dd, gang, sign, th * 180 / np.pi, track))
        if "scale" not in res:
            print("  (chua co bai xoay tay nen TRACK tinh voi GYRO_SCALE = 1)")
    else:
        print("Lan chay cuoi khong phai bai motor xoay (SW3) - bam SW3 SAU bai xoay tay.")
    if res:
        print()
        print("---- chep vao logic_code/core/config.h ----")
        if "sign" in res:
            print("constexpr float GYRO_SIGN  = %+.1ff;" % res["sign"])
        if "scale" in res:
            print("constexpr float GYRO_SCALE = %.4ff;" % res["scale"])
        if "track" in res:
            print("constexpr float TRACK_MM   = %.2ff;" % res["track"])
        st.update({"gyro_" + k: float(v) for k, v in res.items()})
        save_state(st)
    return res


def cmd_gyro(st, binfile=None):
    global DT
    DT = 0.004
    if binfile:
        w = list(np.fromfile(binfile, dtype=np.uint32))
        hdr = w[:RUNLOG_HDR]
        ln = hdr[RUNLOG_HDR - 1]
        log = parse_log(w[RUNLOG_HDR:RUNLOG_HDR + LOG_WORDS * ln]) if ln > 4 else None
    else:
        hdr = read_words(RUNLOG_ADDR, RUNLOG_HDR)
        if hdr[0] != RUNLOG_MAGIC or hdr[1] == 0:
            sys.exit("Robot chua luu lan do nao.")
        ln = hdr[RUNLOG_HDR - 1]
        log = parse_log(read_words(RUNLOG_ADDR + 4 * RUNLOG_HDR, LOG_WORDS * ln)) if ln > 4 else None
    if log is not None:
        plot(log, st, "gyro_xoay", "Bai gyro - lan chay cuoi")
    return gyro_analyze(st, hdr, log)


def cmd_ident(st):
    need(st, "dir_ok", "Chay 'python tune.py dir' truoc.")
    print("Dat robot o dau ban, mat huong doc ban (can ~350 mm phia truoc). Robot chay toi roi lui ve.")
    steps = [(100.0, 60.0), (200.0, 80.0), (300.0, 80.0)]
    T_rows, T_y, R_rows, R_y = [], [], [], []
    runs = [("toi", trans_prm(st, steps, 1500.0, 20.0, 20.0, err=60.0)),
            ("lui", trans_prm(st, [(-v, h) for v, h in steps], 1500.0, 20.0, 20.0, err=60.0))]
    for name, prm in runs:
        stt, log = run_cmd(CMD_PROFILE, prm, "tinh tien " + name)
        if log is not None:
            plot(log, st, "ident_" + name, "Do mo hinh - tinh tien " + name)
        if stt != 2:
            sys.exit("Dung giua chung - xem do thi tools/output/ident_%s.png" % name)
        sl, sr, ul, ur = states(log, st)
        sat = np.maximum(np.abs(ul), np.abs(ur)) > DUTY_MAX * 0.97
        r, y = window_rows(0.5 * (sl + sr), 0.5 * (ul + ur), sat)
        T_rows += r
        T_y += y
    arc = 36.0 * np.pi   # ~180 do neu TRACK ~72 mm (chua can dung)
    # 2 toc do xoay de tach duoc KS (ma sat) voi KV (ti le toc do)
    for name, dist, v in (("trai_cham", arc, 150.0), ("phai_cham", -arc, 150.0),
                          ("trai_nhanh", arc, 300.0), ("phai_nhanh", -arc, 300.0)):
        stt, log = run_cmd(CMD_PROFILE, rot_prm(st, dist, v, 1500.0, 20.0, 20.0, err=60.0), "xoay " + name)
        if log is not None:
            plot(log, st, "ident_xoay_" + name, "Do mo hinh - xoay " + name)
        if stt != 2:
            sys.exit("Dung giua chung khi xoay - xem do thi tools/output/ident_xoay_%s.png" % name)
        sl, sr, ul, ur = states(log, st)
        sat = np.maximum(np.abs(ul), np.abs(ur)) > DUTY_MAX * 0.97
        r, y = window_rows(0.5 * (sr - sl), 0.5 * (ur - ul), sat)
        R_rows += r
        R_y += y
    (ks, kv, ka), r2 = fit(T_rows, T_y, "tinh tien")
    (ks_r, kv_r, ka_r), r2r = fit(R_rows, R_y, "xoay     ")
    if kv <= 0 or ka <= 0 or ka_r <= 0 or r2 < 0.8:
        sys.exit("Ket qua khong hop ly - gui tớ do thi trong tools/output de xem.")
    st.update({"ks": float(max(ks, 0.0)), "kv": float(kv), "ka": float(ka),
               "kv_r": float(kv_r), "ka_r": float(ka_r), "ks_rot": float(ks_r), "ident_ok": True})
    save_state(st)
    print("\nROT_INERTIA = KA_xoay / KA = %.2f ; KV xoay / KV tinh tien = %.2f ; KS xoay %.3f vs %.3f"
          % (ka_r / ka, kv_r / kv, ks_r, ks))
    print_config(st)


def run_pair(st, wn, wn_r, kind):
    """2 lan chay doi chieu de robot ve gan cho cu"""
    out = []
    if kind == "trans":
        steps = [(450.0, 120.0)]
        for sgn in (1, -1):
            prm = trans_prm(st, [(sgn * v, h) for v, h in steps], 3000.0, wn, wn_r)
            stt, log = run_cmd(CMD_PROFILE, prm, "wn %.0f %s" % (wn, "toi" if sgn > 0 else "lui"))
            out.append((stt, log))
    else:
        for sgn in (1, -1):
            prm = rot_prm(st, sgn * 36.0 * np.pi / 2, 300.0, 3000.0, wn, wn_r)
            stt, log = run_cmd(CMD_PROFILE, prm, "wn_r %.0f %s" % (wn_r, "trai" if sgn > 0 else "phai"))
            out.append((stt, log))
    return out


def summarize(results, st, key):
    ms = [metrics(log, st) for stt, log in results if log is not None]
    ok = all(stt == 2 for stt, _ in results) and len(ms) == len(results)
    if not ms:
        return None
    agg = {k: max(m[k] for m in ms) for k in ("max_e", "max_ed", "over", "chatter", "final")}
    agg["ok"] = ok
    print("     -> sai so tinh tien max %.2f mm, sai so xoay max %.2f mm/banh, vot lo %.2f mm, rung duty %.3f%s"
          % (agg["max_e"], agg["max_ed"], agg["over"], agg["chatter"], "" if ok else "  (BI DUNG)"))
    return agg


def cmd_verify(st, wn=None, wn_r=None):
    need(st, "ident_ok", "Chay 'python tune.py ident' truoc.")
    wn = wn or st["wn"]
    wn_r = wn_r or st["wn_r"]
    print("Chay thu PID: WN_POS = %.1f, WN_ANG = %.1f rad/s" % (wn, wn_r))
    for kind in ("trans", "rot"):
        res = run_pair(st, wn, wn_r, kind)
        summarize(res, st, kind)
        if res[0][1] is not None:
            plot(res[0][1], st, "verify_" + kind, "PID wn %.0f / wn_r %.0f - %s" % (wn, wn_r, kind))


def cmd_sweep(st):
    need(st, "ident_ok", "Chay 'python tune.py ident' truoc.")
    print("Dat robot o dau ban (can ~250 mm phia truoc). Robot chay toi-lui va xoay qua lai nhieu lan.")
    best = {}
    for kind, cands in (("trans", [6.0, 9.0, 12.0, 16.0, 20.0, 25.0]), ("rot", [10.0, 15.0, 20.0, 26.0, 33.0])):
        good = []
        for w in cands:
            wn, wn_r = (w, st["wn_r"]) if kind == "trans" else (st["wn"], w)
            agg = summarize(run_pair(st, wn, wn_r, kind), st, kind)
            if agg is None or not agg["ok"]:
                break
            err = agg["max_e"] if kind == "trans" else agg["max_ed"]
            stable = agg["over"] < 1.0 and agg["chatter"] < 0.03
            if not stable:
                print("     (dao dong / rung -> dung tang)")
                break
            good.append((err, w))
        if not good:
            sys.exit("Khong co bang thong nao on dinh - gui tớ do thi.")
        top = max(w for _, w in good)
        pick = round(0.8 * top, 1)   # chua 20% du tru cho pin yeu, san khac
        best[kind] = pick
        print("  %s: on dinh toi %.0f rad/s -> chon %.1f\n" % (kind, top, pick))
    st["wn"], st["wn_r"] = best["trans"], best["rot"]
    save_state(st)
    cmd_verify(st)
    print_config(st)


def print_config(st):
    print("\n---- chep vao logic_code/core/config.h ----")
    print("constexpr float KV_MOTOR     = %.5ff;   // V/(mm/s)" % (st["kv"] * VBAT_NOMINAL))
    print("constexpr float KA_MOTOR     = %.6ff;  // V/(mm/s^2)" % (st["ka"] * VBAT_NOMINAL))
    print("constexpr float KS_MOTOR     = %.3ff;     // V" % (st["ks"] * VBAT_NOMINAL))
    print("constexpr float ROT_INERTIA  = %.2ff;" % (st["ka_r"] / st["ka"]))
    print("constexpr float COUNTS_PER_MM_L/R = %.4ff;" % st["cpm"])
    print("constexpr int   MOTOR_SIGN_L = %+d, MOTOR_SIGN_R = %+d;" % (st["msign_l"], st["msign_r"]))
    print("constexpr float WN_POS = %.1ff, WN_ANG = %.1ff;" % (st["wn"], st["wn_r"]))


def main():
    st = load_state()
    if len(sys.argv) < 2:
        print(__doc__)
        return
    c, a = sys.argv[1], sys.argv[2:]
    if c == "dir":
        cmd_dir(st)
    elif c == "straight":
        cmd_straight(st, *(float(x) for x in a))
    elif c == "cpm":
        cmd_cpm(st, float(a[0]))
    elif c == "ident":
        if a and a[0] == "flash":
            cmd_ident_flash(st)
        elif a and a[0] == "file":
            cmd_ident_flash(st, a[1])
        else:
            cmd_ident(st)
    elif c == "walls":
        cmd_walls(st, center=bool(a and a[0] == "center"))
    elif c == "arm":
        cmd_arm(st, a[0] if a else "wall", *(float(x) for x in a[1:]))
    elif c == "runs":
        cmd_runs(st)
    elif c == "events":
        cmd_events(st)
    elif c == "gyro":
        if a and a[0] == "err":
            cmd_gyro_err(st, float(a[1]))
        else:
            cmd_gyro(st, a[1] if len(a) > 1 and a[0] == "file" else None)
    elif c == "verify":
        cmd_verify(st, *(float(x) for x in a))
    elif c == "sweep":
        cmd_sweep(st)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
