#!/usr/bin/env python3
"""Verify Fairino/Tianji native co-loading and FK/IK without connecting hardware."""
import argparse
import importlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import subprocess
import sys

EXPECTED = {
    "numpy": "2.2.4", "pin": "4.1.0", "cmeel-boost": "1.90.0",
    "cmeel-urdfdom": "6.0.0", "cmeel-tinyxml2": "11.0.0",
}

def check_order(order, tianji_source):
    if tianji_source:
        sys.path.insert(0, str(tianji_source.resolve()))
    modules = {name: importlib.import_module(name) for name in order.split(',')}
    fairino = modules['libpyfairino']
    model = fairino.Kinematics()
    options = fairino.Options()
    options.body_avoidance = False
    q = [math.radians(x) for x in [-30,45,0,60,0,0,0,30,45,0,60,0,0,0]]
    left = model.forward_kinematics(q, 'left', options)
    right = model.forward_kinematics(q, 'right', options)
    result = model.solve_ik(left, right, [x + .005 for x in q], options)
    assert result.success, (result.status, result.message)
    tianji = modules['libpytianji']
    tmodel = tianji.PinkIKModel()
    tq = [0.21, -0.37, 0.16, -0.82, 0.19, -0.31, 0.12]
    for arm in (0, 1):
        pose = tmodel.forward_kinematics(arm, tq)
        solved = tmodel.solve(arm, pose, tq, [0.1] * 7)
        assert solved['success'], solved
    mapped = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                     if 'libpinocchio_default.so' in line})
    assert len(mapped) == 1, mapped
    return dict(order=order.split(','), fairino_version=fairino.__version__,
                tianji_api=tianji.API_VERSION, fk_ik='passed', pinocchio_library=mapped[0])

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tianji-source', type=Path, help='Parent of an existing libpytianji package; optional when installed')
    parser.add_argument('--order', help=argparse.SUPPRESS)
    args = parser.parse_args()
    actual = {name: version(name) for name in EXPECTED}
    assert actual == EXPECTED, actual
    if args.order:
        print(json.dumps(check_order(args.order, args.tianji_source)))
        return
    results = []
    for order in ('libpyfairino,libpytianji', 'libpytianji,libpyfairino'):
        cmd = [sys.executable, '-I', '-B', str(Path(__file__).resolve()), '--order', order]
        if args.tianji_source:
            cmd += ['--tianji-source', str(args.tianji_source)]
        result = subprocess.run(cmd, check=True, capture_output=True, text=True, timeout=60)
        results.append(json.loads(result.stdout))
    print(json.dumps(dict(versions=actual, checks=results), indent=2))

if __name__ == '__main__':
    main()
