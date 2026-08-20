import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MONITOR_PATH = (
    ROOT / "src" / "deyes_stereo" / "deyes_stereo" / "runtime_acceptance_monitor.py"
)


def test_finish_if_due_uses_module_json_without_local_import() -> None:
    tree = ast.parse(MONITOR_PATH.read_text(encoding="utf-8"))
    module_json_imports = [
        node
        for node in tree.body
        if isinstance(node, ast.Import)
        and any(alias.name == "json" for alias in node.names)
    ]
    assert module_json_imports, "json must remain available on the module scope"

    finish_methods = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "finish_if_due"
    ]
    assert len(finish_methods) == 1
    assert not any(
        isinstance(node, (ast.Import, ast.ImportFrom))
        and any(
            getattr(alias, "name", "").split(".")[0] == "json"
            for alias in getattr(node, "names", ())
        )
        for node in ast.walk(finish_methods[0])
    ), "finish_if_due must not shadow the module json binding"

    # The no-file path still needs the module binding when the timer callback
    # serializes runtime metrics.
    assert any(
        isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "json"
        and node.attr == "dumps"
        for node in ast.walk(finish_methods[0])
    )
