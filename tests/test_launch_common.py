"""scripts/lib/common.sh: background processes must not outlive a launch script that dies without its EXIT trap."""

import subprocess
import time
from pathlib import Path

COMMON = Path(__file__).resolve().parents[1] / "scripts" / "lib" / "common.sh"


def _gone(pid: int, timeout_s: float = 5.0) -> bool:
    """True once the process has exited (absent, or a zombie waiting for its new parent to reap it)."""
    end = time.time() + timeout_s
    while time.time() < end:
        try:
            state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
        except FileNotFoundError:
            return True
        if state == "Z":
            return True
        time.sleep(0.1)
    return False


def test_start_bg_children_die_with_a_killed_script(tmp_path):
    pid_file = tmp_path / "pid"
    script = f'source "{COMMON}"; start_bg "{tmp_path}/child.log" sleep 300; echo $! > "{pid_file}"; kill -KILL $$'
    subprocess.run(["bash", "-c", script], capture_output=True, timeout=10)
    assert _gone(int(pid_file.read_text()))


def test_start_bag_refuses_an_existing_output(tmp_path):
    (tmp_path / "bag").mkdir()
    result = subprocess.run(
        ["bash", "-c", f'source "{COMMON}"; start_bag "{tmp_path}/bag"'], capture_output=True, text=True, timeout=10
    )
    assert result.returncode != 0
    assert "already exists" in result.stderr
