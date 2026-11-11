"""Runs only native offline model calculations; no Session is constructed."""
import math
from libpyfairino import Kinematics, Options

model = Kinematics()
options = Options()
q = [math.radians(x) for x in [-30,45,0,60,0,0,0,30,45,0,60,0,0,0]]
left = model.forward_kinematics(q, "left", options)
right = model.forward_kinematics(q, "right", options)
result = model.solve_ik(left, right, q, options)
print(result.success, result.status, result.q)
