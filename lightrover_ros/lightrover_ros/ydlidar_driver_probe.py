#!/usr/bin/env python3

import argparse
import collections
import os
import re
import signal
import subprocess
import time


MODEL_RE = re.compile(r'Model:\s+(.+)')
SERIAL_RE = re.compile(r'Serial:\s+(.+)')
SINGLE_FIXED_RE = re.compile(r'Single Fixed Size:\s+(\d+)')
FIXED_RE = re.compile(r'Fixed Size:\s+(\d+)')
SAMPLE_RATE_RE = re.compile(r'Sample Rate:\s+([0-9.]+K)')
SCAN_FREQ_RE = re.compile(r'Scan Frequency:\s+([0-9.]+Hz)')


def summarize_output(output: str) -> str:
    lines = output.splitlines()
    checksum_errors = sum('Check Sum' in line for line in lines)
    timeout_errors = sum('Timeout count' in line for line in lines)
    failed_scans = sum('Failed to get scan' in line for line in lines)
    point_count_warnings = sum('Real point count' in line for line in lines)

    def collect(pattern: re.Pattern[str]) -> list[str]:
        return [match.group(1).strip() for line in lines if (match := pattern.search(line))]

    models = collect(MODEL_RE)
    serials = collect(SERIAL_RE)
    single_fixed = collect(SINGLE_FIXED_RE)
    fixed = collect(FIXED_RE)
    sample_rates = collect(SAMPLE_RATE_RE)
    scan_freqs = collect(SCAN_FREQ_RE)

    interesting = [
        line for line in lines
        if (
            'Lidar successfully connected' in line
            or 'Fail to get baseplate device information' in line
            or 'Now lidar is scanning' in line
            or 'Successed to check the lidar' in line
            or 'Failed to start scan mode' in line
            or 'Error, cannot retrieve Lidar health code' in line
        )
    ]

    lines_out = [
        f'checksum_errors={checksum_errors}',
        f'timeout_errors={timeout_errors}',
        f'failed_scan_errors={failed_scans}',
        f'point_count_warnings={point_count_warnings}',
        f'models={", ".join(models) if models else "none"}',
        f'serials={", ".join(serials) if serials else "none"}',
        f'single_fixed_sizes={", ".join(single_fixed) if single_fixed else "none"}',
        f'fixed_sizes={", ".join(fixed) if fixed else "none"}',
        f'sample_rates={", ".join(sample_rates) if sample_rates else "none"}',
        f'scan_frequencies={", ".join(scan_freqs) if scan_freqs else "none"}',
    ]

    if interesting:
        lines_out.append('key_lines=')
        lines_out.extend(f'  {line}' for line in interesting[:20])

    return '\n'.join(lines_out)


def run_probe(params_file: str, duration: float) -> int:
    command = [
        'ros2',
        'run',
        'ydlidar_ros2_driver',
        'ydlidar_ros2_driver_node',
        '--ros-args',
        '--params-file',
        params_file,
    ]

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )

    time.sleep(duration)
    os.killpg(process.pid, signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        output, _ = process.communicate()

    print(f'params_file={params_file}')
    print(f'duration_sec={duration:.2f}')
    print(summarize_output(output))
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Run ydlidar_ros2_driver briefly and summarize key diagnostics.'
    )
    parser.add_argument(
        '--params-file',
        required=True,
        help='Path to a YDLIDAR ROS parameter YAML file.',
    )
    parser.add_argument('--duration', type=float, default=8.0)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    return run_probe(args.params_file, args.duration)


if __name__ == '__main__':
    raise SystemExit(main())
