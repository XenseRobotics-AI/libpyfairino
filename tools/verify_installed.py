#!/usr/bin/env python3
"""Check an installed wheel from any working directory, without constructing SDK objects."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import subprocess
import libpyfairino as f

parser=argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args=parser.parse_args()
package=Path(f.__file__).resolve().parent
provenance=json.loads((package/'provenance.json').read_text())
matrix=json.loads((package/'support-matrix.json').read_text())['declarations']
for item in matrix:
    if item['phase_one_status']=='supported' and item['owner']=='SingleRobot' and not (item['constructor'] or item['destructor']):
        assert hasattr(f.Arm, item['name']), item['name']
assert not hasattr(f,'RobotTool') and not hasattr(f,'FR_struct_converse')
assert not hasattr(f,'RobotWaist')
expected={
 'libhumanorid_sdk.so.3':'libhumanorid_sdk.so.3.0.0',
 'libfairino_motion_planner.so.1':'libfairino_motion_planner.so.1.0.4',
 'libfairino_middleware_logger.so.1':'libfairino_middleware_logger.so.1.0.0',
 'libkine_FR_7Axis.so':'libkine_FR_7Axis.so.1',
}
for installed, original in expected.items():
    record=next(x for x in provenance['files'] if x['path']=='third_party/official_sdk/lib/'+original)
    assert hashlib.sha256((package/'_vendor'/installed).read_bytes()).hexdigest()==record['sha256']
for item in provenance['files']:
    if item['path'].startswith('libpyfairino/assets/'):
        p=package/item['path'].removeprefix('libpyfairino/')
        assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'], str(p)
model=f.Kinematics()
options=f.Options()
options.body_avoidance=False
q=[math.radians(x) for x in [-30,45,0,60,0,0,0,30,45,0,60,0,0,0]]
left=model.forward_kinematics(q,'left',options)
right=model.forward_kinematics(q,'right',options)
result=model.solve_ik(left,right,[x+.005 for x in q],options)
assert result.success, (result.status,result.message)
model_path=Path(f.model_path()).resolve()
assert model_path.is_relative_to(package)
maps=Path('/proc/self/maps').read_text()
source='/home/olorin/Olorin/02_Fairino/libpyfairino'
assert source not in maps
assert '/home/olorin/下载/' not in maps
assert 'libhumanorid_sdk' not in maps, 'offline import/model loaded transport SDK'
assert '_sdk_backend' not in maps
assert str(package/'_vendor'/'libkine_FR_7Axis.so') in maps
native=next(package.glob('_native*.so'))
elfs={}
for p in [native,package/'_sdk_backend.so',package/'_sdk_worker']:
    dynamic=subprocess.check_output(['readelf','-dW',str(p)],text=True)
    assert '/home/olorin/' not in dynamic and '/tmp/' not in dynamic, dynamic
    elfs[p.name]=dynamic
report=dict(package=str(package),model_path=str(model_path),model_nq=model.nq,full_nq=model.full_nq,full_nv=model.full_nv,
 supported_declarations=sum(x['phase_one_status']=='supported' for x in matrix),
 statuses={s:sum(x['phase_one_status']==s for x in matrix) for s in ['supported','deferred','upstream-incompatible']},
 fk_ik_success=result.success,left_position_error=result.left_position_error,right_position_error=result.right_position_error,
 source_libraries_mapped=False,transport_sdk_loaded_offline=False,
 mapped_libraries=sorted(set(line.split()[-1] for line in maps.splitlines() if '/' in line and ('cmeel.prefix' in line or 'libpyfairino/' in line))),elf_dynamic=elfs)
args.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ['elf_dynamic','mapped_libraries']},indent=2))
