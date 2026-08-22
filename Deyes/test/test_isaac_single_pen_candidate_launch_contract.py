import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_single_pen_candidate_is_namespaced_and_never_physical():
    launch = (ROOT / "src/deyes_bringup/launch/isaac_single_pen_candidate.launch.py").read_text(encoding="utf-8")
    config = (ROOT / "config/stereo/isaac_single_pen_candidate.defaults.yaml").read_text(encoding="utf-8")
    source = (ROOT / "src/deyes_stereo/deyes_stereo/isaac_single_pen_candidate_node.py").read_text(encoding="utf-8")
    assert "isaac_single_pen_candidate" in launch and "ROS_DOMAIN_ID" in launch
    assert "/x1_sim/detection/pen_features" in config and "/x1_sim/grasp/single_pen_candidate" in config
    assert "candidate_count" in source
    assert "physical_execution_eligible" in source
    assert "/dev/right_arm" not in source


def test_single_candidate_rejects_invalid_geometry_before_tf_and_expires_unpaired_inputs():
    source = (ROOT / "src/deyes_stereo/deyes_stereo/isaac_single_pen_candidate_node.py").read_text(encoding="utf-8")
    assert 'if result.get("valid") is not True:' in source
    assert source.index('if result.get("valid") is not True:') < source.index("self._lookup(frame, stamp)")
    assert "self._discard_stale()" in source
    assert '"unpaired_input_expired"' in source

    tree = ast.parse(source)
    helper = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "discard_stale_cache")
    namespace: dict[str, object] = {}
    exec(compile(ast.Module(body=[helper], type_ignores=[]), str(ROOT), "exec"), namespace)
    discard = namespace["discard_stale_cache"]

    cache = {100: "expired", 500: "boundary", 501: "fresh"}
    assert discard(cache, 1_000, 500) == 1
    assert cache == {500: "boundary", 501: "fresh"}
    assert discard(cache, 1_000, -1) == 2
    assert cache == {}
