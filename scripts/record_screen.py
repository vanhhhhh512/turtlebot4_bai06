#!/usr/bin/env python3
"""Record the actual desktop, with Gazebo and laptop webcam side by side."""
import argparse
import time

import cv2
import numpy as np
from PIL import ImageGrab


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--fps', type=float, default=12)
    args = parser.parse_args()
    frame = np.asarray(ImageGrab.grab())
    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'),
                             args.fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f'Cannot create video: {args.output}')
    start = time.monotonic()
    next_frame = start
    count = 0
    try:
        while time.monotonic() - start < args.seconds:
            image = np.asarray(ImageGrab.grab())
            writer.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            count += 1
            next_frame += 1 / args.fps
            time.sleep(max(0, next_frame - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        writer.release()
    print(f'Wrote {count} actual desktop frames to {args.output}')


if __name__ == '__main__':
    main()
