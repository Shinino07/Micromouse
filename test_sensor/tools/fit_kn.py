"""
Tim he so k, n cho 5 cam bien IR:   d[mm] = k * value^(-n)

Lay log 2 ve:  ln(d) = ln(k) - n * ln(value)  -> duong thang tren do thi log-log.
"Ke duong trung binh" = hoi quy tuyen tinh (binh phuong toi thieu) qua cac diem do,
do doc = -n, giao diem = ln(k).

Cach dung:
    python fit_kn.py --template        # tao file sensor_data.xlsx de dien so lieu
    python fit_kn.py                   # doc sensor_data.xlsx, tinh k,n, ve do thi,
                                       # ghi ../Core/Inc/sensor_calib.h
    python fit_kn.py --file khac.xlsx --dmin 30 --dmax 200

File Excel: 5 sheet S1..S5, moi sheet 2 cot: distance_mm | adc
(adc = so in ra o dong CAP cua firmware, cot cua dung sensor dang do).
"""
import argparse
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
DEFAULT_XLSX = HERE / "sensor_data.xlsx"
OUT_DIR = HERE / "output"
HEADER_PATH = HERE.parent / "Core" / "Inc" / "sensor_calib.h"
N_SENSORS = 5
SAT_LEVEL = 3600                      # value >= muc nay coi nhu bao hoa (cong nen ~200 la ~3850)
OUTLIER_PCT = 8.0                     # diem lech duong fit > 8 % -> nen do lai
TEMPLATE_DIST = list(range(20, 220, 10))   # 20 diem: 20, 30, ..., 210 mm


def make_template(path: Path) -> None:
    if path.exists():
        sys.exit(f"{path.name} da ton tai, khong ghi de. Xoa/doi ten truoc neu muon tao lai.")
    with pd.ExcelWriter(path, engine="openpyxl") as xw:
        for s in range(1, N_SENSORS + 1):
            pd.DataFrame({"distance_mm": TEMPLATE_DIST,
                          "adc": [None] * len(TEMPLATE_DIST)}).to_excel(xw, sheet_name=f"S{s}", index=False)
    print(f"Da tao {path}  (5 sheet S1..S5, moi sheet 20 khoang cach 20..210 mm)")


def load_sheet(xls: pd.ExcelFile, name: str) -> pd.DataFrame:
    df = pd.read_excel(xls, sheet_name=name)
    df.columns = [str(c).strip().lower() for c in df.columns]
    if "distance_mm" not in df or "adc" not in df:
        raise ValueError(f"sheet {name} can 2 cot 'distance_mm' va 'adc'")
    df = df[["distance_mm", "adc"]].apply(pd.to_numeric, errors="coerce").dropna()
    return df[(df.distance_mm > 0) & (df.adc > 0)].sort_values("distance_mm").reset_index(drop=True)


def select_points(df: pd.DataFrame, dmin: float, dmax: float):
    """Bo diem bao hoa, diem ngoai [dmin, dmax] va vung qua gan (truoc dinh tin hieu)."""
    keep = (df.adc < SAT_LEVEL) & (df.distance_mm >= dmin) & (df.distance_mm <= dmax)
    reasons = {}
    for i in df.index[df.adc >= SAT_LEVEL]:
        reasons[i] = "bao hoa"
    # Qua gan: tin hieu giam khi lai gan hon -> nam truoc dinh, khong theo luat k*x^-n
    mean_by_d = df.groupby("distance_mm").adc.mean()
    d_peak = mean_by_d.idxmax()
    near = df.distance_mm < d_peak
    for i in df.index[near]:
        reasons.setdefault(i, f"gan hon dinh tin hieu ({d_peak:g} mm)")
    keep &= ~near
    for i in df.index[~keep]:
        reasons.setdefault(i, "ngoai khoang dmin/dmax")
    return df[keep], df[~keep], reasons


def fit(df: pd.DataFrame):
    x = np.log(df.adc.to_numpy(float))
    y = np.log(df.distance_mm.to_numpy(float))
    slope, intercept = np.polyfit(x, y, 1)
    k, n = math.exp(intercept), -slope
    y_hat = intercept + slope * x
    r2 = 1.0 - np.sum((y - y_hat) ** 2) / np.sum((y - y.mean()) ** 2)
    d = df.distance_mm.to_numpy(float)
    d_hat = k * df.adc.to_numpy(float) ** (-n)
    err = d_hat - d
    return dict(k=k, n=n, r2=r2, rmse=float(np.sqrt(np.mean(err ** 2))),
                max_pct=float(np.max(np.abs(err) / d) * 100), slope=slope, intercept=intercept)


def plot_sensor(name, used, dropped, res, out_png):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    fig.suptitle(f"{name}:  d = {res['k']:.1f} * value^(-{res['n']:.4f})    "
                 f"R² = {res['r2']:.4f}, RMSE = {res['rmse']:.1f} mm, sai so max = {res['max_pct']:.1f} %")

    xs = np.linspace(used.adc.min(), used.adc.max(), 200)
    # Log-log: cac diem + duong trung binh (hoi quy)
    a1.scatter(np.log(used.adc), np.log(used.distance_mm), label="diem do", zorder=3)
    if len(dropped):
        a1.scatter(np.log(dropped.adc), np.log(dropped.distance_mm), marker="x", c="gray", label="bi loai")
    a1.plot(np.log(xs), res["intercept"] + res["slope"] * np.log(xs), "r-",
            label=f"duong trung binh: ln d = {res['intercept']:.3f} - {res['n']:.4f}·ln x")
    a1.set_xlabel("ln(value ADC)"); a1.set_ylabel("ln(d mm)"); a1.set_title("Log-log")
    a1.grid(alpha=.3); a1.legend(fontsize=8)

    # Thang thuong: khoang cach theo ADC
    a2.scatter(used.adc, used.distance_mm, label="diem do", zorder=3)
    if len(dropped):
        a2.scatter(dropped.adc, dropped.distance_mm, marker="x", c="gray", label="bi loai")
    a2.plot(xs, res["k"] * xs ** (-res["n"]), "r-", label="d = k·x^(-n)")
    a2.set_xlabel("value ADC"); a2.set_ylabel("khoang cach (mm)"); a2.set_title("Thang thuong")
    a2.grid(alpha=.3); a2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def write_header(results, path: Path):
    ks = ", ".join(f"{results[s]['k']:.6g}f" if s in results else "0.0f" for s in range(N_SENSORS))
    ns = ", ".join(f"{results[s]['n']:.6g}f" if s in results else "0.0f" for s in range(N_SENSORS))
    path.write_text(
        "/* Sinh tu dong boi tools/fit_kn.py - khong sua tay.\n"
        " * d[mm] = IR_K[i] * value^(-IR_N[i]);  he so = 0 -> sensor chua hieu chuan */\n"
        "#ifndef SENSOR_CALIB_H\n#define SENSOR_CALIB_H\n\n"
        f"#define IR_CALIB_VALID {1 if results else 0}\n\n"
        f"static const float IR_K[5] = {{{ks}}};\n"
        f"static const float IR_N[5] = {{{ns}}};\n\n#endif\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description="Tim k, n cho cam bien IR: d = k * value^(-n)")
    ap.add_argument("--file", type=Path, default=DEFAULT_XLSX)
    ap.add_argument("--template", action="store_true", help="tao file Excel mau roi thoat")
    ap.add_argument("--dmin", type=float, default=0, help="chi dung diem >= dmin mm")
    ap.add_argument("--dmax", type=float, default=1e9, help="chi dung diem <= dmax mm")
    ap.add_argument("--no-header", action="store_true", help="khong ghi sensor_calib.h")
    args = ap.parse_args()

    if args.template:
        make_template(args.file)
        return
    if not args.file.exists():
        sys.exit(f"Khong thay {args.file}. Chay 'python fit_kn.py --template' de tao file mau.")

    OUT_DIR.mkdir(exist_ok=True)
    xls = pd.ExcelFile(args.file)
    results, rows, per_point = {}, [], []
    for s in range(N_SENSORS):
        name = f"S{s + 1}"
        if name not in xls.sheet_names:
            continue
        df = load_sheet(xls, name)
        if len(df) == 0:
            print(f"{name}: chua co so lieu, bo qua")
            continue
        used, dropped, reasons = select_points(df, args.dmin, args.dmax)
        for i, why in reasons.items():
            print(f"{name}: loai diem d={df.distance_mm[i]:g} mm, adc={df.adc[i]:g} ({why})")
        if len(used) < 3 or used.adc.nunique() < 2:
            print(f"{name}: can >= 3 diem hop le de tinh, hien co {len(used)}")
            continue
        res = fit(used)
        results[s] = res
        pred = res["k"] * used.adc ** (-res["n"])
        pct = (pred - used.distance_mm) / used.distance_mm * 100
        per_point.append(pd.DataFrame({"sensor": name, "distance_mm": used.distance_mm, "adc": used.adc,
                                       "d_tinh_mm": pred.round(1), "sai_so_%": pct.round(2)}))
        for d, p in zip(used.distance_mm, pct):
            if abs(p) > OUTLIER_PCT:
                print(f"{name}: diem d={d:g} mm lech {p:+.1f} % so voi duong fit -> nen do lai diem nay")
        plot_sensor(name, used, dropped, res, OUT_DIR / f"{name}.png")
        rows.append([name, len(used), res["k"], res["n"], res["r2"], res["rmse"], res["max_pct"]])
        if res["r2"] < 0.98:
            print(f"{name}: R² = {res['r2']:.3f} thap -> kiem tra lai diem do (ghi nham, tuong nghieng...)")

    if not results:
        sys.exit("Khong sensor nao du so lieu.")

    table = pd.DataFrame(rows, columns=["sensor", "so diem", "k", "n", "R2", "RMSE_mm", "sai_so_max_%"])
    print("\n" + table.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    table.to_csv(OUT_DIR / "ket_qua_kn.csv", index=False)
    pd.concat(per_point).to_csv(OUT_DIR / "sai_so_tung_diem.csv", index=False)

    # Do thi tong hop 5 sensor
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for s, res in results.items():
        df = load_sheet(xls, f"S{s + 1}")
        used, _, _ = select_points(df, args.dmin, args.dmax)
        xs = np.linspace(used.adc.min(), used.adc.max(), 200)
        line, = ax.plot(xs, res["k"] * xs ** (-res["n"]), label=f"S{s + 1}: k={res['k']:.1f}, n={res['n']:.3f}")
        ax.scatter(used.adc, used.distance_mm, s=12, color=line.get_color())
    ax.set_xlabel("value ADC"); ax.set_ylabel("khoang cach (mm)")
    ax.set_title("Tong hop: d = k·value^(-n)"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(OUT_DIR / "tong_hop.png", dpi=120); plt.close(fig)

    if not args.no_header:
        write_header(results, HEADER_PATH)
        print(f"\nDa ghi {HEADER_PATH}  -> build lai firmware de in ra mm")
    print(f"Do thi + bang ket qua: {OUT_DIR}")


if __name__ == "__main__":
    main()
