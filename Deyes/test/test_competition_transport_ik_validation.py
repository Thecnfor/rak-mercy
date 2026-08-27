from pathlib import Path
from deyes_stereo.competition_transport_ik_validation import validate_official_urdf_solution

URDF=Path(r"E:\a_robot\temp\deyes\mercury_x1_ros2-official-20260820\src\mercury_robot_urdf\mercury_robot_urdf\urdf\mercury_x1\mercury_x1.urdf")
SOLUTION=[0.9039235121922143,1.0947778286517673,-0.5541657632173106,-1.4630624027485444,0.5699777942163569,1.297798402557579]

def test_official_urdf_transport_solution_fk_and_limits():
    if not URDF.exists(): return
    result=validate_official_urdf_solution(URDF,SOLUTION)
    assert result["validated"] and result["joint_limits_passed"]
    assert result["position_residual_mm"]<=5 and result["orientation_residual_deg"]<=2
