#!/usr/bin/env python3

import argparse
import collections
import os
import statistics
import termios
import time


BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    500000: termios.B500000,
    576000: termios.B576000,
    921600: termios.B921600,
}


def configure_serial(fd: int, baudrate: int) -> None:
    if baudrate not in BAUD_MAP:
        supported = ', '.join(str(rate) for rate in sorted(BAUD_MAP))
        raise ValueError(f'Unsupported baudrate {baudrate}. Supported: {supported}')

    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNPAR
    attrs[1] = 0
    attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
    attrs[3] = 0
    attrs[4] = BAUD_MAP[baudrate]
    attrs[5] = BAUD_MAP[baudrate]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcflush(fd, termios.TCIOFLUSH)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_bytes(fd: int, duration_sec: float, chunk_size: int) -> bytes:
    deadline = time.monotonic() + duration_sec
    chunks = []

    while time.monotonic() < deadline:
        data = os.read(fd, chunk_size)
        if data:
            chunks.append(data)

    return b''.join(chunks)


def find_offsets(data: bytes, pattern: bytes) -> list[int]:
    offsets = []
    start = 0
    while True:
        idx = data.find(pattern, start)
        if idx < 0:
            return offsets
        offsets.append(idx)
        start = idx + 1


def summarize_offsets(offsets: list[int]) -> str:
    if len(offsets) < 2:
        return 'not enough hits'

    gaps = [offsets[i + 1] - offsets[i] for i in range(len(offsets) - 1)]
    common = collections.Counter(gaps).most_common(5)
    common_text = ', '.join(f'{gap}x{count}' for gap, count in common)
    return (
        f'mean_gap={statistics.mean(gaps):.1f} '
        f'min_gap={min(gaps)} max_gap={max(gaps)} '
        f'common={common_text}'
    )


def probe(device: str, baudrate: int, duration_sec: float, chunk_size: int, preview_len: int) -> int:
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY)
    try:
        configure_serial(fd, baudrate)
        data = read_bytes(fd, duration_sec, chunk_size)
    finally:
        os.close(fd)

    aa55 = find_offsets(data, b'\xaa\x55')
    fiftyfiveaa = find_offsets(data, b'\x55\xaa')
    byte_counter = collections.Counter(data)

    print(f'device={device}')
    print(f'baudrate={baudrate}')
    print(f'duration_sec={duration_sec:.2f}')
    print(f'bytes_read={len(data)}')
    print(f'aa55_hits={len(aa55)} ({summarize_offsets(aa55)})')
    print(f'55aa_hits={len(fiftyfiveaa)} ({summarize_offsets(fiftyfiveaa)})')

    top_bytes = ', '.join(
        f'0x{byte_value:02x}:{count}' for byte_value, count in byte_counter.most_common(8)
    )
    print(f'top_bytes={top_bytes}')

    preview = data[:preview_len].hex(' ')
    print(f'preview={preview}')

    if aa55:
        preview_offsets = ', '.join(str(offset) for offset in aa55[:10])
        print(f'aa55_first_offsets={preview_offsets}')
    if fiftyfiveaa:
        preview_offsets = ', '.join(str(offset) for offset in fiftyfiveaa[:10])
        print(f'55aa_first_offsets={preview_offsets}')

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Probe raw YDLIDAR serial bytes without using the ROS driver.'
    )
    parser.add_argument('--device', default='/dev/ydlidar')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--chunk-size', type=int, default=512)
    parser.add_argument('--preview-len', type=int, default=128)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    return probe(
        device=args.device,
        baudrate=args.baudrate,
        duration_sec=args.duration,
        chunk_size=args.chunk_size,
        preview_len=args.preview_len,
    )


if __name__ == '__main__':
    raise SystemExit(main())
