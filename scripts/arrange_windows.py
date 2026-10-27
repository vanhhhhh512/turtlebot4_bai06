#!/usr/bin/env python3
"""Put Gazebo beside the laptop webcam window for a screen recording."""
import os
import shutil
import subprocess
import time
from pathlib import Path


def tool_and_env():
    tool = shutil.which('xdotool')
    env = os.environ.copy()
    if tool:
        return tool, env
    tool = Path('/home/admin1/xla/wk4/week5/tools/local/usr/bin/xdotool')
    if tool.is_file():
        env['LD_LIBRARY_PATH'] = (str(tool.parent.parent / 'lib/x86_64-linux-gnu')
                                  + ':' + env.get('LD_LIBRARY_PATH', ''))
        return str(tool), env
    raise RuntimeError('xdotool is unavailable')


def main():
    tool, env = tool_and_env()

    def run(*args):
        return subprocess.run([tool, *map(str, args)], env=env, text=True,
                              capture_output=True)

    dimensions = run('getdisplaygeometry')
    if dimensions.returncode:
        raise RuntimeError('X11 display is unavailable')
    width, height = map(int, dimensions.stdout.split())
    left = round(width * 0.60)
    layouts = [('Gazebo', (0, 0, left, height)),
               ('Bai 06 - laptop webcam', (left, 0, width - left, height))]
    moved = set()
    while True:
        for title, (x, y, w, h) in layouts:
            found = run('search', '--onlyvisible', '--name', title)
            if found.returncode:
                continue
            for window in found.stdout.splitlines():
                if window.isdigit() and window not in moved:
                    run('windowsize', window, w, h)
                    run('windowmove', window, x, y)
                    run('windowraise', window)
                    moved.add(window)
        time.sleep(2)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Window arrangement unavailable: {exc}', flush=True)
