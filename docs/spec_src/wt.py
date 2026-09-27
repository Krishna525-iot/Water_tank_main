"""Bench test helper: reads / changes the running firmware's state over the
ST-Link (SWD, hot-plug - the board keeps running).

  py -3 wt.py state                 current mode, motor, level, sensors, faults
  py -3 wt.py fast [sec]            shorten dry-run test, testing gap (default 60 s)
                                    and max run (2 min) in RAM only - a reboot
                                    brings back the saved settings
  py -3 wt.py set <field> <value>   RAM write: dry_test_s, gap_s, maxrun_min,
                                    retry, dry_en, uv, ov
  py -3 wt.py time HH:MM[:SS] [dow] set the RTC (dow 1=Sun..7=Sat, default today)
  py -3 wt.py slot n HH:MM HH:MM    timer slot n (1-5), all days, enabled (RAM)
  py -3 wt.py slotoff n             disable timer slot n (RAM)
  py -3 wt.py twist ON OFF [HH:MM HH:MM]   twist minutes and optional window (RAM)
"""
import datetime
import re
import struct
import subprocess
import sys

PLUG = r"C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins"
CLI = PLUG + r"\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe"
NM = PLUG + r"\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-nm.exe"
ELF = r"D:\test\stm32_water_tank-main\stm32_water_tank-main\Debug\WT1.1.elf"

WANT = ["motorStatus", "manualActive", "semiAutoActive", "timerActive", "countdownActive",
        "twistActive", "autoActive", "restartActive", "autoPausedByUser", "manualPausedByUser",
        "countdownMode", "semiTankFullHold",
        "manualMotorOff", "motorOwner", "dryState", "groundWater", "senseDryRun",
        "senseOverLoad", "senseUnderLoad", "senseOverUnderVolt", "senseMaxRunReached",
        "countdownDuration", "twist_on_phase", "twist_deadline", "twistFilledLatch",
        "twistDryFailed", "autoState", "timerState", "timerStateDeadline", "stateDeadline",
        "time", "sys", "uwTick", "timerSlots", "twistSettings", "g_voltageV", "g_currentA",
        "adcData", "cdDefaultMin", "bootStartBlockUntil", "autoFilledLatch", "auto_retry_count",
        "timer_retry_count"]
OWNER = ["NONE", "MANUAL", "SEMIAUTO", "TIMER", "COUNTDOWN", "TWIST", "AUTO", "REFILL"]
AUTO_ST = ["IDLE", "WAIT", "GAP-WAIT", "RUNNING"]   # AUTO_IDLE, ON_WAIT, DRY_CHECK, OFF_WAIT
TIMER_ST = ["TEST", "GAP-WAIT", "RUNNING"]
DRY_ST = ["-", "TESTING", "FAULT"]


def symbols():
    out = subprocess.run([NM, "-S", ELF], capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and p[0].startswith("2000"):
            name = re.sub(r"\.\d+$", "", p[3])
            syms.setdefault(name, (int(p[0], 16), int(p[1], 16)))
    # stable tank level lives in a function-local static
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and p[3].startswith("stableLevel"):
            syms["stableLevel"] = (int(p[0], 16), int(p[1], 16))
    return syms


def cli(*args):
    r = subprocess.run([CLI, "-c", "port=SWD", "mode=HOTPLUG", *args], capture_output=True, text=True)
    if "Error" in r.stdout:
        err = [l for l in r.stdout.splitlines() if "Error" in l]
        sys.exit("ST-Link: " + " / ".join(err))
    return r.stdout


def read_ram(start, length):
    out = cli("-r8", hex(start), hex(length))
    data = bytearray()
    for line in out.splitlines():
        m = re.match(r"^0x([0-9A-Fa-f]{8})\s*:\s*(.*)$", line.strip())
        if m:
            data += bytes(int(b, 16) for b in m.group(2).split())
    return bytes(data)


def w8(addr, val):
    cli("-w8", hex(addr), hex(val & 0xFF))


def w16(addr, val):
    cli("-w16", hex(addr), hex(val & 0xFFFF))


def w32(addr, val):
    cli("-w32", hex(addr), hex(val & 0xFFFFFFFF))


def snapshot(syms):
    used = {k: v for k, v in syms.items() if k in WANT or k == "stableLevel"}
    lo = min(a for a, _ in used.values())
    hi = max(a + s for a, s in used.values())
    raw = read_ram(lo, hi - lo)

    def b(name, off=0, n=1):
        a, _ = used[name]
        return raw[a - lo + off: a - lo + off + n]

    def u8(name, off=0):
        return b(name, off)[0] if name in used else 0

    def u32(name, off=0):
        return struct.unpack("<I", b(name, off, 4))[0] if name in used else 0

    def f32(name):
        return struct.unpack("<f", b(name, 0, 4))[0] if name in used else 0.0

    return used, raw, lo, u8, u32, f32, b


def state(syms):
    used, raw, lo, u8, u32, f32, b = snapshot(syms)
    tick = u32("uwTick")
    modes = [n for n, k in (("MANUAL", "manualActive"), ("SEMI", "semiAutoActive"), ("TIMER", "timerActive"),
                            ("COUNTDOWN", "countdownActive"), ("TWIST", "twistActive"), ("AUTO", "autoActive"),
                            ("REFILL", "restartActive")) if u8(k)]
    if u8("countdownMode") and not u8("countdownActive"):
        modes.append("COUNTDOWN(ended)")
    if u8("semiAutoActive") and u8("semiTankFullHold"):
        modes.append("(semi run ended)")
    paused = [n for n, k in (("AUTO(off by button)", "autoPausedByUser"),
                             ("MANUAL(off by button)", "manualPausedByUser")) if u8(k)]
    faults = [n for n, k in (("OVERLOAD", "senseOverLoad"), ("UNDERLOAD", "senseUnderLoad"),
                             ("VOLTAGE", "senseOverUnderVolt"), ("MAX-RUN", "senseMaxRunReached")) if u8(k)]
    t = b("time", 0, 8)
    v = [struct.unpack("<f", b("adcData", 12 + 4 * i, 4))[0] for i in range(6)]
    level = u8("stableLevel")

    def left(dl):
        return "-" if dl == 0 else f"{max(0, (dl - tick) // 1000 if dl >= tick else 0)} s"

    s = u8("sys", 24)
    gap = u32("sys", 0); maxrun = struct.unpack("<H", b("sys", 20, 2))[0]; tgap = struct.unpack("<H", b("sys", 22, 2))[0]
    print(f"MOTOR {'ON ' if u8('motorStatus') else 'OFF'} | mode: {', '.join(modes) or 'none'}"
          f"{' | ' + ', '.join(paused) if paused else ''} | owner {OWNER[u8('motorOwner') % 8]}")
    print(f"LEVEL {level}% | G.W {'YES' if u8('groundWater') else 'NO '} | dry sensor "
          f"{'WATER' if u8('senseDryRun') else 'NO WATER'} | dry state {DRY_ST[u8('dryState') % 3]}"
          f" | faults: {', '.join(faults) or 'none'}")
    print(f"auto: {AUTO_ST[u8('autoState') % 4]} next {left(u32('stateDeadline'))}, retries {u8('auto_retry_count')}"
          f", filled-latch {u8('autoFilledLatch')} | timer: {TIMER_ST[u8('timerState') % 3]} next "
          f"{left(u32('timerStateDeadline'))}, retries {u8('timer_retry_count')}")
    if u8("countdownActive"):
        print(f"countdown: {u32('countdownDuration')} s left (button time {u8('cdDefaultMin')} min)")
    if u8("twistActive"):
        print(f"twist: {'ON' if u8('twist_on_phase') else 'OFF'} period, {left(u32('twist_deadline'))} left"
              f"{', dry failed' if u8('twistDryFailed') else ''}{', waiting for 50%' if u8('twistFilledLatch') else ''}")
    print(f"clock {t[2]:02d}:{t[1]:02d}:{t[0]:02d} dow {t[3]} {t[4]:02d}-{t[5]:02d}-{t[6] | (t[7] << 8)}"
          f" | {f32('g_voltageV'):.0f} V {f32('g_currentA'):.2f} A"
          f" | settings: dry {'ON' if s else 'off'} test {gap} s, gap {tgap} s, max run {maxrun} min"
          f" | uptime {tick // 1000} s")
    print("probes V: " + " ".join(f"{n}={x:.2f}" for n, x in zip(("100", "75", "50", "25", "DRY", "GW"), v)))


def main():
    syms = symbols()
    cmd = sys.argv[1] if len(sys.argv) > 1 else "state"
    a = sys.argv[2:]
    sysaddr = syms["sys"][0]
    if cmd == "state":
        state(syms)
    elif cmd == "fast":
        sec = int(a[0]) if a else 60
        w32(sysaddr + 0, sec)          # dry-run test window
        w16(sysaddr + 22, sec)         # testing gap
        w16(sysaddr + 20, 2)           # max run 2 min
        print(f"RAM only: dry test {sec} s, testing gap {sec} s, max run 2 min")
        state(syms)
    elif cmd == "set":
        f, v = a[0], int(a[1])
        off = {"dry_test_s": (0, 32), "retry": (4, 8), "uv": (6, 16), "ov": (8, 16),
               "maxrun_min": (20, 16), "gap_s": (22, 16), "dry_en": (24, 8)}[f]
        {8: w8, 16: w16, 32: w32}[off[1]](sysaddr + off[0], v)
        print(f"RAM only: {f} = {v}")
    elif cmd == "time":
        hh, mm, *ss = [int(x) for x in a[0].split(":")]
        now = datetime.date.today()
        dow = int(a[1]) if len(a) > 1 else (now.isoweekday() % 7) + 1
        req = syms["g_rtcSetRequest"][0]
        w32(req, (ss[0] if ss else 0) | (mm << 8) | (hh << 16) | (dow << 24))
        w32(req + 4, now.day | (now.month << 8) | ((now.year - 2000) << 16) | (0xA5 << 24))
        print(f"clock set to {hh:02d}:{mm:02d} dow {dow}")
    elif cmd == "slot":
        n = int(a[0]); on = [int(x) for x in a[1].split(":")]; off = [int(x) for x in a[2].split(":")]
        base = syms["timerSlots"][0] + (n - 1) * 6
        for i, val in enumerate((on[0], on[1], off[0], off[1], 0x7F, 1)):
            w8(base + i, val)
        print(f"RAM only: slot {n} {a[1]}-{a[2]} all days, enabled")
    elif cmd == "slotoff":
        w8(syms["timerSlots"][0] + (int(a[0]) - 1) * 6 + 5, 0)
        print(f"RAM only: slot {a[0]} disabled")
    elif cmd == "twist":
        base = syms["twistSettings"][0]
        w16(base, int(a[0]) * 60); w16(base + 2, int(a[1]) * 60)
        if len(a) >= 4:
            on = [int(x) for x in a[2].split(":")]; off = [int(x) for x in a[3].split(":")]
        else:
            on = off = [0, 0]
        for i, val in enumerate((on[0], on[1], off[0], off[1])):
            w8(base + 4 + i, val)
        print(f"RAM only: twist ON {a[0]} min / OFF {a[1]} min, window {a[2:] or 'all day'}")
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
