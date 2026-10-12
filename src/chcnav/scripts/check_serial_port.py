#!/usr/bin/env python3
"""Reject a second reader of the same Linux serial device (including aliases)."""
import os
from pathlib import Path
import stat
import sys


def port_users(port, proc_root=Path('/proc')):
    device = os.stat(port)
    if not stat.S_ISCHR(device.st_mode):
        raise ValueError(f'{port} 不是字符设备')
    users = []
    for process in proc_root.iterdir():
        if not process.name.isdigit():
            continue
        try:
            descriptors = list((process / 'fd').iterdir())
        except (FileNotFoundError, PermissionError):
            continue
        for descriptor in descriptors:
            try:
                opened = descriptor.stat()
            except (FileNotFoundError, PermissionError):
                continue
            if stat.S_ISCHR(opened.st_mode) and opened.st_rdev == device.st_rdev:
                try:
                    name = (process / 'comm').read_text().strip()
                except (FileNotFoundError, PermissionError):
                    name = 'unknown'
                users.append((int(process.name), name))
                break
    return sorted(users)


def main():
    try:
        users = port_users(sys.argv[1])
    except (OSError, ValueError) as error:
        print(f'[失败] 串口占用检查失败：{error}', file=sys.stderr)
        return 1
    if users:
        print(f'[失败] 华测串口 {sys.argv[1]} 已被占用：' +
              ', '.join(f'PID={pid} ({name})' for pid, name in users), file=sys.stderr)
        print('重复启动会争读串口，导致报文缺字、CRC/XOR 校验失败及 odometry 断流。'
              '请在原终端停止已有链路，或在总启动脚本中加 --reuse-ins 复用。', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
