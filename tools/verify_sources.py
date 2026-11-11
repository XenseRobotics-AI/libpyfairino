#!/usr/bin/env python3
"""Verify the exact copied source bytes and baseline without changing either repository."""
from pathlib import Path
import hashlib,json,subprocess
root=Path(__file__).resolve().parents[1]
data=json.loads((root/'third_party/provenance.json').read_text())
for entry in data['files']:
    for p in [root/entry['path'],Path(entry['source'])]:
        assert hashlib.sha256(p.read_bytes()).hexdigest()==entry['sha256'],str(p)
head=subprocess.check_output(['git','-C',data['ik_source'],'rev-parse','HEAD'],text=True).strip()
assert head==data['ik_commit']
print(json.dumps(dict(verified_files=len(data['files']),ik_commit=head,official_source=data['official_sdk_source']),indent=2))
