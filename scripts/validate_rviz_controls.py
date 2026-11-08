#!/usr/bin/env python3
"""Exercise camera controls only in temporary RViz test windows (X11)."""
import ctypes
import json
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

import yaml
from PIL import ImageGrab

ROOT = Path(__file__).resolve().parents[1]


def main():
    x = ctypes.CDLL('libX11.so.6')
    xt = ctypes.CDLL('libXtst.so.6')
    pointer = ctypes.c_void_p
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x.XOpenDisplay.restype = pointer
    x.XRaiseWindow.argtypes = [pointer, ctypes.c_ulong]
    x.XSetInputFocus.argtypes = [pointer, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    x.XFlush.argtypes = [pointer]
    x.XCloseDisplay.argtypes = [pointer]
    x.XStringToKeysym.argtypes = [ctypes.c_char_p]
    x.XStringToKeysym.restype = ctypes.c_ulong
    x.XKeysymToKeycode.argtypes = [pointer, ctypes.c_ulong]
    x.XKeysymToKeycode.restype = ctypes.c_ubyte
    xt.XTestFakeMotionEvent.argtypes = [pointer, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
    xt.XTestFakeButtonEvent.argtypes = [pointer, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
    xt.XTestFakeKeyEvent.argtypes = [pointer, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
    display = x.XOpenDisplay(None)
    if not display:
        raise RuntimeError('An authorized X11 DISPLAY is required')
    output = ROOT / 'generated/rviz_controls'
    output.mkdir(parents=True, exist_ok=True)
    reports = {}

    def flush():
        x.XFlush(display)
        time.sleep(0.15)

    def move(px, py):
        xt.XTestFakeMotionEvent(display, -1, int(px), int(py), 0)
        flush()

    def button(number, down):
        xt.XTestFakeButtonEvent(display, number, int(down), 0)
        flush()

    def key(name, down):
        code = x.XKeysymToKeycode(display, x.XStringToKeysym(name.encode()))
        xt.XTestFakeKeyEvent(display, code, int(down), 0)
        flush()

    try:
        for profile in ['debug', 'perception']:
            config = output / f'controls_{profile}.rviz'
            shutil.copyfile(ROOT / 'src/baja_perception_sim/config' / f'{profile}.rviz', config)
            before = yaml.safe_load(config.read_text())['Visualization Manager']['Views']['Current']
            log = (output / f'{profile}.log').open('w')
            process = subprocess.Popen(['/opt/ros/humble/lib/rviz2/rviz2', '-d', str(config),
                                        '--ros-args', '-r', f'__node:=controls_{profile}_rviz'],
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                window = None
                for _ in range(100):
                    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
                    for line in tree.splitlines():
                        if config.name in line and 'RViz' in line:
                            window = int(re.search(r'0x[0-9a-f]+', line).group(), 16)
                    if window is not None:
                        break
                    if process.poll() is not None:
                        raise RuntimeError(f'RViz exited; see {log.name}')
                    time.sleep(0.1)
                if window is None:
                    raise RuntimeError('Cannot find temporary RViz test window')
                time.sleep(2)
                info = subprocess.check_output(['xwininfo', '-id', hex(window)], text=True)
                def value(name):
                    return int(re.search(rf'{name}:\s+(-?\d+)', info).group(1))
                # RViz docks are on the left; on high-DPI displays they can
                # occupy over half the window. Exercise the right-hand 3D panel.
                px = value('Absolute upper-left X') + value('Width') * 0.85
                py = value('Absolute upper-left Y') + value('Height') * 0.55
                x.XRaiseWindow(display, window)
                x.XSetInputFocus(display, window, 2, 0)
                flush()
                bx,by=value('Absolute upper-left X'),value('Absolute upper-left Y')
                ImageGrab.grab(bbox=(bx,by,bx+value('Width'),by+value('Height'))).save(output/f'{profile}_window.png')
                move(px, py)
                button(1, True)
                for step in range(1, 6):
                    move(px + step * 15, py + step * 5)
                button(1, False)
                move(px, py)
                button(4, True)
                button(4, False)
                button(2, True)
                move(px + 40, py + 20)
                button(2, False)
                key('Control_L', True)
                key('s', True)
                key('s', False)
                key('Control_L', False)
                time.sleep(0.5)
                after = yaml.safe_load(config.read_text())['Visualization Manager']['Views']['Current']
                checks = {'rotate': abs(after['Yaw']-before['Yaw']) > 0.01,
                          'zoom': abs(after['Distance']-before['Distance']) > 0.01,
                          'pan': after['Focal Point'] != before['Focal Point']}
                reports[profile] = {'checks': checks, 'before': before, 'after': after}
                print(json.dumps({profile: checks}), flush=True)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                log.close()
    finally:
        x.XCloseDisplay(display)
        (output / 'report.json').write_text(json.dumps(reports, indent=2))
    if not all(all(r['checks'].values()) for r in reports.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
