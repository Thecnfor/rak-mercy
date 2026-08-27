import pytest
from deyes_stereo.competition_pick_execution import Mercury650Executor, MotionProfile
from deyes_stereo.competition_grasp_verification import GraspVerifier

class FakeMercury:
    def __init__(self,status=None,offset=None): self.pose=[0]*6; self.calls=[]; self.status=status or [0]*6; self.offset=offset or [0]*6
    def send_base_coords(self,pose,speed): self.pose=list(pose); self.calls.append(("move",tuple(pose),speed))
    def get_base_coords(self): return [a+b for a,b in zip(self.pose,self.offset)]
    def get_robot_status(self): return self.status
    def set_gripper_value(self,value,speed): self.calls.append(("gripper",value,speed))

def test_exact_pick_sequence_direction_speeds_feedback_and_single_latch():
    fake=FakeMercury(); ex=Mercury650Executor(fake,MotionProfile(transport_validated=True))
    trace=ex.pick(401,9)
    assert [item[0] for item in fake.calls]==["gripper"]+["move"]*4+["gripper"]+["move"]*3
    assert [c[1][2] for c in fake.calls if c[0]=="move"]==[235,180,140,135,180,235,260]
    assert [c[2] for c in fake.calls if c[0]=="move"]==[8,8,5,5,8,8,8]
    assert all(c[1][0]==401 and c[1][1]==9 for c in fake.calls if c[0]=="move" and c[1][2]!=260)
    assert trace[-1]["phase"]=="transport"
    with pytest.raises(RuntimeError,match="already_latched"): ex.pick(401,9)

def test_place_sequence_and_direction():
    fake=FakeMercury(); Mercury650Executor(fake,MotionProfile(transport_validated=True)).place(390,-5)
    assert [c[1][2] for c in fake.calls if c[0]=="move"]==[200,165,200,260,260]
    assert [c[2] for c in fake.calls if c[0]=="move"]==[8,5,8,8,8]

def test_status_feedback_and_unvalidated_transport_fail_closed_no_retry():
    with pytest.raises(RuntimeError,match="robot_status_not_ok"):
        Mercury650Executor(FakeMercury(status=[0,0,1]),MotionProfile(transport_validated=True)).pick(400,10)
    fake=FakeMercury()
    with pytest.raises(RuntimeError,match="transport_pose_not_ik_validated"):
        Mercury650Executor(fake,MotionProfile(transport_validated=False)).pick(400,10)
    assert len([c for c in fake.calls if c[0]=="move"])==6

def test_pose_error_and_timeout_are_enforced():
    fake=FakeMercury(offset=[6,0,0,0,0,0])
    def one_shot_wait(robot,expected,timeout):
        actual=robot.get_base_coords()
        if max(abs(actual[i]-expected[i]) for i in range(3))>5: raise RuntimeError("pose_timeout")
    with pytest.raises(RuntimeError,match="pose_timeout"):
        Mercury650Executor(fake,MotionProfile(transport_validated=True),waiter=one_shot_wait).pick(400,10)

def test_grasp_verification_a_or_b_failure_blocks_nav_and_latches():
    assert GraspVerifier(10).verify(pen_height_over_table_m=.03,original_roi_has_pen=[True]*3,gripper_feedback=10)["success"]
    b=GraspVerifier(10).verify(pen_height_over_table_m=None,original_roi_has_pen=[False]*3,gripper_feedback=15)
    assert b["success"] and b["navigation_permitted"]
    verifier=GraspVerifier(10); fail=verifier.verify(pen_height_over_table_m=.029,original_roi_has_pen=[False,False],gripper_feedback=20)
    assert not fail["success"] and not fail["navigation_permitted"]
    assert verifier.verify(pen_height_over_table_m=.1,original_roi_has_pen=[False]*3,gripper_feedback=20)["success"] is False
