import math
import pytest
import libpyfairino as f

def test_packaged_native_fk_ik_and_modes():
    m=f.Kinematics();o=f.Options();o.body_avoidance=False
    q=[math.radians(x) for x in [-30,45,0,60,0,0,0,30,45,0,60,0,0,0]]
    left=m.forward_kinematics(q,"left",o);right=m.forward_kinematics(q,"right",o)
    seed=[v+.005 for v in q]
    result=m.solve_ik(left,right,seed,o)
    assert result.success, (result.status,result.message,result.error)
    assert result.left_position_error<1e-4 and result.right_position_error<1e-4
    assert len(result.q)==14 and len(result.full_q)==32
    assert m.full_nq==32 and m.full_nv==28
    assert m.joint_names==["j"+str(i) for i in range(1,15)]
    torso=f.Kinematics(active_torso=True)
    assert torso.nq==19 and torso.joint_names[-5:]==["j20","j19","j18","j16","j15"]
    torso_q=q+[0.]*5
    assert len(torso.forward_kinematics(torso_q,"left",o))==16
    o.torso_position=[.05,0.,0.,0.,0.]
    assert m.forward_kinematics(q,"left",o)!=left
    flange=f.Kinematics(left_frame="left_flange",right_frame="right_flange")
    assert flange.forward_kinematics(q,"left",f.Options())!=left

def test_world_base_and_invalid_input():
    m=f.Kinematics();raw=f.Kinematics(base_pose=f.URDF_BASE_POSE);o=f.Options()
    q=[0.]*14
    a=m.forward_kinematics(q,"left",o);b=raw.forward_kinematics(q,"left",o)
    assert a[3]==pytest.approx(-b[7]) and a[7]==pytest.approx(b[3])
    with pytest.raises((ValueError,RuntimeError)):m.forward_kinematics([0.]*13,"left",o)
    with pytest.raises((ValueError,RuntimeError)):m.forward_kinematics([float("nan")]+[0.]*13,"left",o)
    with pytest.raises((ValueError,RuntimeError)):m.forward_kinematics(q,"unknown",o)
    with pytest.raises((ValueError,RuntimeError)):m.solve_ik([float("nan")]*16,[0.]*16,q,o)

def test_official_math_types_and_real_forward():
    d=f.sdk_math.DHParam();comp=f.sdk_math.DHCMPSTPARAM()
    d.DH_a=[.1]*7;d.DH_d=[.01]*7
    r=f.sdk_math.forwardKinematic(d,comp,[0.]*7)
    assert r.code==0
    assert [r.transform_mm[i] for i in [3,7,11]]==pytest.approx([700,0,70])
    with pytest.raises(TypeError):d.DH_a=[0.]*6
    with pytest.raises(ValueError):d.DH_a=[float("inf")]*7
    with pytest.raises(ValueError):f.sdk_math.forwardKinematic(d,comp,[0.]*7,5)
    limits=f.sdk_math.JOINTLIMIT()
    with pytest.raises(ValueError):f.sdk_math.inverseKinematic(d,comp,limits,7,0,f.URDF_BASE_POSE,[0.]*7)


def test_real_vendor_inverse_preserves_code_and_output_validity():
    # Synthetic DH, deliberately not a claim about ART7 geometry.
    d=f.sdk_math.DHParam();comp=f.sdk_math.DHCMPSTPARAM()
    d.DH_d=[.187,0.,.29,0.,.31,0.,.1]
    d.DH_alpha=[0.,-math.pi/2,math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,math.pi/2]
    q=[20.,30.,-10.,50.,10.,30.,5.]
    fk=f.sdk_math.forwardKinematic(d,comp,q)
    assert fk.code==0 and math.isfinite(fk.arm_angle_deg)
    limits=f.sdk_math.JOINTLIMIT()
    for j in range(1,8):setattr(limits,"joint"+str(j),[-180.,180.])
    result=f.sdk_math.inverseKinematic(d,comp,limits,7,fk.arm_angle_deg,fk.transform_mm,q)
    assert len(result.q_deg)==7 and len(result.all_solutions_deg)==16
    assert all(len(row)==7 for row in result.all_solutions_deg)
    assert result.output_finite==all(math.isfinite(x) for x in result.q_deg)
    assert result.code==0 and not result.output_finite  # observed upstream behavior, not fake success
    with pytest.raises(ValueError,match="last row"):
        f.sdk_math.inverseKinematic(d,comp,limits,7,fk.arm_angle_deg,[0.]*16,q)
