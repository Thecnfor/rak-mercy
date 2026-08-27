from pathlib import Path
import yaml

ROOT=Path(__file__).parents[1]
def test_galactic_parameter_file_has_only_node_ros_parameters_and_site_is_split():
    params=yaml.safe_load((ROOT/"config/stereo/competition_fixed_scene.yaml").read_text())
    assert all(list(value)==["ros__parameters"] for value in params.values())
    site=yaml.safe_load((ROOT/"config/stereo/competition_venue_65cm.yaml").read_text())
    assert set(site["fallbacks"])=={"fixed_height","bbox_center","fixed_xy"}
    assert site["table_height_m"]==.650 and site["reference_table_height_m"]==.560
    assert site["transport"]["transport_validated"] is True
    assert site["transport"]["joint_limits_passed"] is True
