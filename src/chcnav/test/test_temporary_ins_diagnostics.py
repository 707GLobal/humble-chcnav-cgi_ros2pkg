"""Exercise the built parser with silent and malformed inputs, never real serial."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import pytest

ROOT = Path(__file__).resolve().parents[3]
BINARY = ROOT / 'install/chcnav/lib/chcnav/HcMsgParserLaunchNode'


@pytest.mark.parametrize('contents', [b'', b'\x00' * 100])
def test_silent_input_and_bad_frames_are_distinguishable(tmp_path, contents):
    data = tmp_path / 'input.bin'
    data.write_bytes(contents)
    log = tmp_path / 'driver.log'
    environment = dict(os.environ, ROS_DOMAIN_ID='197', CHCNAV_DIAGNOSTIC_DIR=str(tmp_path))
    with log.open('w') as stream:
        process = subprocess.Popen([str(BINARY), '--ros-args', '-p', 'type:=file',
                                    '-p', 'path:=' + str(data), '-p', 'rate:=1000'],
                                   env=environment, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 5
            rows = []
            while time.monotonic() < deadline and process.poll() is None:
                file = tmp_path / 'parser.jsonl'
                if file.exists():
                    rows = [json.loads(line) for line in file.read_text().splitlines() if line]
                if len(rows) >= 2:
                    break
                time.sleep(.1)
            assert len(rows) >= 2, log.read_text()
            assert rows[-1]['total']['bytes'] == len(contents)
            assert rows[-1]['total']['empty_reads'] > 0
            assert rows[-1]['total']['cgi_frames'] == 0
            assert rows[-1]['total']['parse_errors'] == len(contents)
            assert rows[-1]['delta']['bytes'] == 0
            assert rows[-1]['input_age_ms'] == -1 if not contents else rows[-1]['input_age_ms'] > 0
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
