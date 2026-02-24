#!/usr/bin/env python3
import argparse
import math
import select
import subprocess
import sys
import termios
import tty


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def read_key(timeout: float = 0.05):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        seq = ch + sys.stdin.read(2)
        if seq == "\x1b[A": return "w"
        if seq == "\x1b[B": return "s"
        if seq == "\x1b[D": return "a"
        if seq == "\x1b[C": return "d"
        return None
    return ch


def send_pose(world: str, model: str, x: float, y: float, z: float, yaw: float):
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    req = (
        f'name: "{model}", '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
        f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}'
    )
    cmd = [
        "gz", "service",
        "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", req,
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode == 0, (proc.stdout.strip() or proc.stderr.strip())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world",    default="carucior_world")
    parser.add_argument("--model",    default="walking_person")
    parser.add_argument("--x",        type=float, default=0.0)
    parser.add_argument("--y",        type=float, default=-9.5)
    parser.add_argument("--z",        type=float, default=0.0)
    parser.add_argument("--yaw",      type=float, default=1.57)
    parser.add_argument("--step",     type=float, default=0.25)
    parser.add_argument("--yaw-step", type=float, default=0.12)
    args = parser.parse_args()

    x, y, z, yaw = args.x, args.y, args.z, args.yaw

    print("W/↑ inainte  S/↓ inapoi  A/← rotire stanga  D/→ rotire dreapta  R reset  P poza  ESC iesire")

    ok, msg = send_pose(args.world, args.model, x, y, z, yaw)
    if not ok:
        print(f"[WARN] set_pose initial failed: {msg}")

    with RawTerminal():
        while True:
            key = read_key()
            if key is None:
                continue

            if key in ("\x1b", "\x03"):
                print("\r\nIesire.")
                break

            elif key in ("w", "W"):
                x += args.step * math.cos(yaw)
                y += args.step * math.sin(yaw)

            elif key in ("s", "S"):
                x -= args.step * math.cos(yaw)
                y -= args.step * math.sin(yaw)

            elif key in ("a", "A"):
                yaw += args.yaw_step
                yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

            elif key in ("d", "D"):
                yaw -= args.yaw_step
                yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

            elif key in ("r", "R"):
                x, y, z, yaw = args.x, args.y, args.z, args.yaw

            elif key in ("p", "P"):
                print(f"\r\nx={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg")
                continue

            else:
                continue

            ok, msg = send_pose(args.world, args.model, x, y, z, yaw)
            if not ok:
                print(f"\r\n[ERR] set_pose failed: {msg}")


if __name__ == "__main__":
    main()
