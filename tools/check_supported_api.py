#!/usr/bin/env python3
"""Compile/link real selected declarations only. Never executes vendor code."""
from pathlib import Path
import argparse, hashlib, json, re, subprocess, tempfile

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path,required=True);args=parser.parse_args()
matrix=json.loads((root/"docs/support-matrix.json").read_text())["declarations"]
for x in matrix:
    header=root/"third_party/official_sdk"/x["header"].removeprefix("SDK/")
    assert hashlib.sha256(header.read_bytes()).hexdigest()==x["sha256"],str(header)
selected=[x for x in matrix if x["phase_one_status"]=="supported"]
lines=['#include "humanorid_sdk.h"']
for i,x in enumerate(selected):
    pars=[];names=[]
    for p in x["parameter_list"]:
        raw=p["raw"].split("=",1)[0].replace("RobotName ","SingleRobot::RobotName ").strip()
        pars.append(raw);names.append(p["name"])
    decl=", ".join(pars);actual=", ".join(names)
    if x["constructor"]:lines.append(f'SingleRobot* probe_{i}({decl}){{return new SingleRobot({actual});}}')
    elif x["destructor"]:lines.append(f'void probe_{i}(SingleRobot* s){{s->~SingleRobot();}}')
    elif x["owner"]:lines.append(f'void probe_{i}(SingleRobot* s'+(', '+decl if decl else '')+f'){{s->SingleRobot::{x["name"]}({actual});}}')
    else:lines.append(f'void probe_{i}({decl}){{{x["name"]}({actual});}}')
lines.append('int main(){return 0;}')
with tempfile.TemporaryDirectory(prefix="libpyfairino-link-") as d:
    d=Path(d);source=d/"selected.cpp";source.write_text("\n".join(lines))
    sdk=root/"third_party/official_sdk"
    command=["g++","-std=c++17","-I"+str(sdk/"include"),str(source),"-L"+str(sdk/"lib"),"-l:libhumanorid_sdk.so.3","-l:libkine_FR_7Axis.so","-l:libfairino_motion_planner.so.1","-l:libfairino_middleware_logger.so.1","-Wl,-rpath-link,"+str(sdk/"lib"),"-o",str(d/"selected")]
    result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    report=dict(scope="selected declarations, compile/link only; no SDK execution",selected_declarations=len(selected),total_declarations=len(matrix),command=command,exit_code=result.returncode,output=result.stdout)
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))
    raise SystemExit(result.returncode)
