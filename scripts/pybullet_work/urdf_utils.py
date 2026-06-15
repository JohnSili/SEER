import atexit
from pathlib import Path

import paths  # noqa: F401  -- puts REPO_ROOT on sys.path for the import below

from evaluation.collision_guard import _urdf_with_resolved_meshes

_URDF_TEMP_FILES: list[Path] = []


def prepare_urdf_path(urdf_path, mesh_root=None):
    """Resolve package:// mesh paths the same way as teleop_arm SelfCollisionGuard."""
    urdf_path = Path(urdf_path)
    mesh_root = Path(mesh_root) if mesh_root is not None else urdf_path.parent

    load_path, temp_path = _urdf_with_resolved_meshes(urdf_path, mesh_root)
    if temp_path is not None:
        _URDF_TEMP_FILES.append(temp_path)
        atexit.register(lambda p=temp_path: p.unlink(missing_ok=True))
    return load_path
