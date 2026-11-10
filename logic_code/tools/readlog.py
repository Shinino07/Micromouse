"""Doc log va ban do tu robot (firmware logic_code) qua ST-Link - chay sau khi robot chay xong, cam ST-Link vao.

  python readlog.py            log tu lan khoi dong co chay gan nhat + ban do da do
  python readlog.py all        toan bo log trong flash
  python readlog.py map        chi ve ban do
  python readlog.py clear      xoa log (robot tu tao lai lan khoi dong sau)

Log nam o flash sector 10 (0x080C0000), ban do o sector 11 (0x080E0000): cam/rut day lam robot reset nhung khong mat.
"""
import re
import subprocess
import sys

PLUG = r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
CLI = PLUG + r"\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
SWD = ["-c", "port=SWD", "mode=HOTPLUG", "freq=100"]   # day SWD noi dai: 100 kHz moi on dinh
LOG_BASE, LOG_SIZE, LOG_MAGIC = 0x080C0000, 0x20000, 0x31474F4C
MAZE_BASE, MAZE_SIZE, MZ_MAGIC, MZ_ERASED = 0x080E0000, 0x20000, 0x4D5A3136, 0x4D5A0000
N = 16
REC_WORDS = 3 + N * N // 4   # magic, seq, crc, 256 byte o


def cli(args):
    r = subprocess.run([CLI] + SWD + args, capture_output=True, text=True, timeout=180)
    if re.search(r"Error[: ]", r.stdout):
        sys.exit("Loi ST-Link (cam lai day SWD, robot phai co dien):\n" + r.stdout[-400:])
    return r.stdout


def read_words(addr, n):
    args = []
    CH = 4096
    for i in range(0, n, CH):
        args += ["-r32", hex(addr + 4 * i), str(4 * min(CH, n - i))]
    out = cli(args)
    words = {}
    for m in re.finditer(r"0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{8}[ \t]*)+)", out):
        base = int(m.group(1), 16)
        for k, w in enumerate(m.group(2).split()):
            words[base + 4 * k] = int(w, 16)
    return [words.get(addr + 4 * i, 0xFFFFFFFF) for i in range(n)]


def read_log():
    if read_words(LOG_BASE, 1)[0] != LOG_MAGIC:
        return ""
    data = bytearray()
    for k in range(LOG_SIZE // 16384):
        w = read_words(LOG_BASE + 16384 * k, 4096)
        for i, x in enumerate(w):
            if k == 0 and i == 0:
                continue   # magic
            if x == 0xFFFFFFFF:
                return data.replace(b"\x00", b"").decode("ascii", "replace")
            data += x.to_bytes(4, "little")
    return data.replace(b"\x00", b"").decode("ascii", "replace")


def read_maze():
    best = None
    total = MAZE_SIZE // 4
    w = []
    for k in range(0, total, 4096):
        chunk = read_words(MAZE_BASE + 4 * k, 4096)
        w += chunk
        # dung khi gap ban ghi trong
        i = 0
        stop = False
        while i + REC_WORDS <= len(w):
            if w[i] == 0xFFFFFFFF:
                stop = True
                break
            i += REC_WORDS
        if stop:
            break
    i = 0
    while i + REC_WORDS <= len(w) and w[i] != 0xFFFFFFFF:
        magic, seq, crc = w[i], w[i + 1], w[i + 2]
        cells = b"".join(x.to_bytes(4, "little") for x in w[i + 3:i + REC_WORDS])
        if magic == MZ_ERASED:
            best = None
        elif magic == MZ_MAGIC:
            h = 2166136261
            for c in cells:
                h = ((h ^ c) * 16777619) & 0xFFFFFFFF
            if h == crc:
                best = (seq, cells)
        i += REC_WORDS
    return best


def draw_maze(cells):
    # bit d: tuong huong d (0 B, 1 D, 2 N, 3 T); bit 4 + d: da biet huong d
    def wall(x, y, d):
        v = cells[y * N + x]
        return bool(v & (1 << d)), bool(v & (16 << d))
    lines = []
    for y in range(N - 1, -1, -1):
        top = "+"
        mid = ""
        for x in range(N):
            w, k = wall(x, y, 0)
            top += ("---" if w else ("   " if k else " . ")) + "+"
        for x in range(N):
            w, k = wall(x, y, 3)
            mid += "|" if w else (" " if k else ":")
            v = cells[y * N + x]
            mid += " G " if 7 <= x <= 8 and 7 <= y <= 8 else (" S " if x == 0 and y == 0 else
                                                             ("   " if (v & 0xF0) == 0xF0 else " ? "))
        w, k = wall(N - 1, y, 1)
        mid += "|" if w else (" " if k else ":")
        lines.append(top)
        lines.append(mid)
    bot = "+"
    for x in range(N):
        w, k = wall(x, 0, 2)
        bot += ("---" if w else ("   " if k else " . ")) + "+"
    lines.append(bot)
    return "\n".join(lines)


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else ""
    if arg == "clear":
        cli(["-w32", hex(LOG_BASE), "0x00000000"])   # pha magic -> lan khoi dong sau tu xoa sector
        print("Da danh dau xoa log (robot xoa han o lan khoi dong sau).")
        return
    if arg != "map":
        text = read_log()
        if not text:
            print("Chua co log (robot chua chay firmware logic_code?).")
        else:
            lines = text.splitlines()
            if arg != "all":
                # bat dau tu lan khoi dong gan nhat ma sau do co chay
                boots = [i for i, l in enumerate(lines) if "=== KHOI DONG" in l]
                start = 0
                for b in reversed(boots):
                    if any("bat dau luot" in l or "FAULT" in l for l in lines[b:]):
                        start = b
                        break
                lines = lines[start:]
                print("---- log tu lan khoi dong co chay gan nhat (python readlog.py all = toan bo) ----")
            else:
                print("---- toan bo log ----")
            # gop cac dong khoi dong lien tiep (cam/rut ST-Link lam robot reset nhieu lan)
            out, k = [], 0
            while k < len(lines):
                j = k
                while j < len(lines) and "=== KHOI DONG" in lines[j]:
                    j += 1
                if j - k >= 2:
                    out.append(lines[k])
                    out.append("            ... %d lan khoi dong lien tiep (thuong do cam/rut day ST-Link)" % (j - k))
                    k = j
                else:
                    out.append(lines[k])
                    k += 1
            print("\n".join(out))
    best = read_maze()
    print()
    if best is None:
        print("Chua co ban do trong flash.")
    else:
        print("---- ban do da luu (lan %d)  --- = tuong, trong = biet khong co tuong, . : = chua biet, ? = o chua di ----"
              % best[0])
        print(draw_maze(best[1]))


if __name__ == "__main__":
    main()
