# mujoco is a compiled extension without type information; treat its attributes as Any in the editor.
from typing import Any

def __getattr__(name: str) -> Any: ...
