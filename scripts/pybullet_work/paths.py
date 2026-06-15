import sys
from pathlib import Path

# Two levels up from this directory is the repository root, where the
# `evaluation` package lives. Defined once here so scene_config / collision_utils
# / urdf_utils don't each re-derive REPO_ROOT or re-touch sys.path.
REPO_ROOT = Path(__file__).resolve().parents[2]

if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))
