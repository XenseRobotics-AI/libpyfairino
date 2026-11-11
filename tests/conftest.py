from pathlib import Path
import subprocess
import pytest
import libpyfairino as f

ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture(scope="session")
def fake_backend(tmp_path_factory):
    path = tmp_path_factory.mktemp("explicit-fake-sdk") / "test_sdk_backend.so"
    subprocess.run(["g++", "-std=c++17", "-shared", "-fPIC", "-pthread",
        "-I"+str(ROOT/"src"), "-I"+str(ROOT/"third_party/official_sdk/include"),
        str(ROOT/"src/backend.cpp"), str(ROOT/"tests/fake_sdk.cpp"), "-o", str(path)], check=True)
    return str(path)

@pytest.fixture
def config():
    return f.HardwareConfig(f.SDKRobotType.ART3_R7, [-180.]*14, [180.]*14,
        list(range(7))*2, [1.]*14, [0.]*14, [1.]*14,
        "TEST DOUBLE ONLY: no ART7 applicability claim", "synthetic identity test mapping",
        rpc_timeout_ms=150, worker_watchdog_ms=100)
