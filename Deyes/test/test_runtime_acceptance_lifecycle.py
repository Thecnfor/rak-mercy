import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_PATH = (
    ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "runtime_acceptance_monitor.py"
)


def _function(tree: ast.AST, name: str) -> ast.FunctionDef:
    matches = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    assert len(matches) == 1
    return matches[0]


def test_report_callback_marks_finished_without_shutdown() -> None:
    tree = ast.parse(MONITOR_PATH.read_text(encoding="utf-8"))
    finish = _function(tree, "finish_if_due")

    assert any(
        isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute) and target.attr == "finished"
            for target in node.targets
        )
        for node in ast.walk(finish)
    )
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "rclpy"
        and node.func.attr == "shutdown"
        for node in ast.walk(finish)
    )


def test_main_owns_spin_and_shutdown_lifecycle() -> None:
    tree = ast.parse(MONITOR_PATH.read_text(encoding="utf-8"))
    main = _function(tree, "main")

    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "spin_once"
        for node in ast.walk(main)
    )
    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "destroy_node"
        for node in ast.walk(main)
    )
    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "rclpy"
        and node.func.attr == "shutdown"
        for node in ast.walk(main)
    )
