from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pybullet as pb


def parse_csv_names(s: str) -> list[str]:
    return [x.strip() for x in s.split(",") if x.strip()]


def parse_ignore_pairs(s: str) -> set[frozenset[str]]:
    """
    CSV список пар:
        linkA:linkB,linkC:linkD
    """
    out: set[frozenset[str]] = set()

    for tok in s.split(","):
        item = tok.strip()

        if not item:
            continue

        if ":" not in item:
            raise ValueError(
                f"Некорректная пара '{item}', ожидается формат linkA:linkB"
            )

        a, b = (x.strip() for x in item.split(":", 1))

        if not a or not b or a == b:
            raise ValueError(f"Некорректная пара '{item}'")

        out.add(frozenset((a, b)))

    return out


def _is_arm_link(name: str) -> bool:
    return (
        name.startswith("arm1_")
        or name.startswith("arm2_")
        or name.startswith("arm_right_")
        or name.startswith("arm_left_")
    )


def _is_arm_mount_link(name: str) -> bool:
    return name in (
        "arm1_base_link",
        "arm2_base_link",
        "arm_right_base_link",
        "arm_left_base_link",
    )


def _resolve_mesh_share_root(
    urdf_path: Path,
    override: Path | None,
) -> Path | None:
    """
    Ищет корень пакета/модели с meshes.
    """

    if override is not None:
        o = override.resolve()

        if not any(
            (o / sub).is_dir()
            for sub in ("meshes", "meshes_bin", "meshes_obj")
        ):
            raise RuntimeError(
                f"--collision-mesh-share: в {o} "
                f"нет meshes/, meshes_bin/ или meshes_obj/"
            )

        return o

    u = urdf_path.resolve()

    if u.parent.name == "urdf":
        share = u.parent.parent

        if any(
            (share / sub).is_dir()
            for sub in ("meshes", "meshes_bin", "meshes_obj")
        ):
            return share

    return None


def _urdf_with_resolved_meshes(
    urdf_path: Path,
    mesh_share: Path | None,
) -> tuple[Path, Path | None]:
    """
    Делает временный URDF с абсолютными путями mesh.
    """

    text = urdf_path.read_text(encoding="utf-8")

    if "<!DOCTYPE" in text:
        text = "\n".join(
            line
            for line in text.splitlines()
            if not line.lstrip().startswith("<!DOCTYPE")
        )

    xml_root = ET.fromstring(text)

    if xml_root.get("version") == "1.0.0":
        xml_root.set("version", "1.0")

    for mj in list(xml_root.findall("mujoco")):
        xml_root.remove(mj)

    search_dirs: list[Path] = []

    if mesh_share is not None:
        share = mesh_share.resolve()

        search_dirs.append(share)

        for sub in ("meshes", "meshes_bin", "meshes_obj"):
            p = share / sub

            if p.is_dir():
                search_dirs.append(p)

    search_dirs.append(urdf_path.parent.resolve())

    changed = False

    for mesh in xml_root.findall(".//mesh"):
        filename = (mesh.get("filename") or "").strip()

        if not filename:
            continue

        resolved: Path | None = None

        if filename.startswith("package://"):
            rel = filename.split("package://", 1)[1]
            rel = rel.split("/", 1)[1]

            if mesh_share is not None:
                resolved = (mesh_share / rel).resolve()

        elif filename.startswith("file://"):
            resolved = Path(filename[7:]).resolve()

        elif filename.startswith("/"):
            resolved = Path(filename).resolve()

        else:
            for d in search_dirs:
                cand = (d / filename).resolve()

                if cand.is_file():
                    resolved = cand
                    break

        if resolved is None or not resolved.is_file():
            raise RuntimeError(
                f"Не удалось найти mesh '{filename}' "
                f"для URDF {urdf_path}"
            )

        new_filename = str(resolved)

        if new_filename != filename:
            mesh.set("filename", new_filename)
            changed = True

    normalized = ET.tostring(
        xml_root,
        encoding="unicode",
    )

    if not changed and normalized == text:
        return urdf_path, None

    fd, tmp_name = tempfile.mkstemp(
        prefix="collision_guard_",
        suffix=".urdf",
        text=True,
    )

    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(normalized)

    except Exception:
        try:
            os.unlink(tmp_name)
        except OSError:
            pass

        raise

    return Path(tmp_name), Path(tmp_name)


def load_disabled_pairs_from_srdf(
    path: Path,
    include_default_arm_pairs: bool = False,
) -> set[frozenset[str]]:
    """
    Загружает disable_collisions пары из SRDF.
    """

    pairs: set[frozenset[str]] = set()

    root = ET.parse(path).getroot()

    for x in root.findall(".//disable_collisions"):
        a = x.attrib.get("link1")
        b = x.attrib.get("link2")
        reason = (x.attrib.get("reason") or "").strip()

        if not a or not b or a == b:
            continue

        if reason == "Default" and not include_default_arm_pairs:

            if (_is_arm_link(a) or _is_arm_link(b)) and not (
                (a == "base_link" and _is_arm_mount_link(b))
                or (b == "base_link" and _is_arm_mount_link(a))
            ):

                if _is_arm_link(a) and _is_arm_link(b):
                    pairs.add(frozenset((a, b)))
                    continue

                non_arm = b if _is_arm_link(a) else a

                if non_arm == "base_link":
                    continue

                pairs.add(frozenset((a, b)))
                continue

        pairs.add(frozenset((a, b)))

    return pairs


class SelfCollisionGuard:
    """
    Self-collision evaluator через:
    - URDF
    - SRDF
    - PyBullet
    """

    def __init__(
        self,
        urdf_path: Path,
        srdf_path: Path,
        right_joint_names: list[str],
        left_joint_names: list[str],
        collision_distance: float = 0.0,
        include_default_arm_pairs: bool = False,
        extra_ignored_pairs: set[frozenset[str]] | None = None,
        mesh_share_root: Path | None = None,
    ) -> None:

        if not urdf_path.exists():
            raise RuntimeError(f"URDF не найден: {urdf_path}")

        if not srdf_path.exists():
            raise RuntimeError(f"SRDF не найден: {srdf_path}")

        self._urdf_temp: Path | None = None

        load_path = urdf_path

        share = _resolve_mesh_share_root(
            urdf_path,
            mesh_share_root,
        )

        urdf_text = urdf_path.read_text(encoding="utf-8")

        needs_rewrite = (
            share is not None
            or "package://" in urdf_text
            or "<!DOCTYPE" in urdf_text
            or "<mujoco>" in urdf_text
        )

        if needs_rewrite:
            load_path, self._urdf_temp = _urdf_with_resolved_meshes(
                urdf_path,
                share,
            )

        self.client_id = pb.connect(pb.DIRECT)

        urdf_flags = (
            pb.URDF_USE_SELF_COLLISION
            | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        )

        self.robot_id = pb.loadURDF(
            str(load_path),
            useFixedBase=True,
            physicsClientId=self.client_id,
            flags=urdf_flags,
        )

        self.disabled_pairs = load_disabled_pairs_from_srdf(
            srdf_path,
            include_default_arm_pairs=include_default_arm_pairs,
        )

        if extra_ignored_pairs:
            self.disabled_pairs |= set(extra_ignored_pairs)

        self.collision_distance = float(
            max(0.0, collision_distance)
        )

        self.joint_name_to_idx: dict[str, int] = {}
        self.link_idx_to_name: dict[int, str] = {}

        body_info = pb.getBodyInfo(
            self.robot_id,
            physicsClientId=self.client_id,
        )

        base_name = (
            body_info[0].decode(errors="ignore")
            if body_info and body_info[0]
            else "base_link"
        )

        self.link_idx_to_name[-1] = base_name

        n = pb.getNumJoints(
            self.robot_id,
            physicsClientId=self.client_id,
        )

        for i in range(n):
            ji = pb.getJointInfo(
                self.robot_id,
                i,
                physicsClientId=self.client_id,
            )

            joint_name = ji[1].decode(errors="ignore")
            link_name = ji[12].decode(errors="ignore")

            self.joint_name_to_idx[joint_name] = i
            self.link_idx_to_name[i] = link_name

        self.right_idxs = [
            self._joint_idx_or_fail(name)
            for name in right_joint_names
        ]

        self.left_idxs = [
            self._joint_idx_or_fail(name)
            for name in left_joint_names
        ]

    def _joint_idx_or_fail(self, name: str) -> int:
        if name not in self.joint_name_to_idx:
            raise RuntimeError(
                f"Joint '{name}' не найден в URDF"
            )

        return self.joint_name_to_idx[name]

    def close(self) -> None:
        if self.client_id >= 0:
            pb.disconnect(
                physicsClientId=self.client_id,
            )

            self.client_id = -1

        if self._urdf_temp is not None:
            try:
                self._urdf_temp.unlink(missing_ok=True)

            except OSError:
                pass

            self._urdf_temp = None

    def _apply_arm_state(
        self,
        idxs: list[int],
        q: list[float],
    ) -> None:

        if len(q) != len(idxs):
            raise RuntimeError(
                f"Длина q={len(q)} "
                f"не совпадает с joint list={len(idxs)}"
            )

        for i, val in zip(idxs, q):
            pb.resetJointState(
                self.robot_id,
                i,
                float(val),
                physicsClientId=self.client_id,
            )

    def set_joint_state(
        self,
        joint_name: str,
        value: float,
    ) -> None:

        idx = self._joint_idx_or_fail(joint_name)

        pb.resetJointState(
            self.robot_id,
            idx,
            float(value),
            physicsClientId=self.client_id,
        )

    def check(
        self,
        q_right: list[float],
        q_left: list[float],
    ) -> tuple[bool, dict]:

        self._apply_arm_state(
            self.right_idxs,
            q_right,
        )

        self._apply_arm_state(
            self.left_idxs,
            q_left,
        )

        cps = pb.getClosestPoints(
            self.robot_id,
            self.robot_id,
            distance=self.collision_distance,
            physicsClientId=self.client_id,
        )

        min_distance = float("inf")

        for cp in cps:
            a_idx = int(cp[3])
            b_idx = int(cp[4])

            if a_idx == b_idx:
                continue

            a = self.link_idx_to_name.get(
                a_idx,
                str(a_idx),
            )

            b = self.link_idx_to_name.get(
                b_idx,
                str(b_idx),
            )

            if frozenset((a, b)) in self.disabled_pairs:
                continue

            dist = float(cp[8])

            min_distance = min(
                min_distance,
                dist,
            )

            if dist <= self.collision_distance:
                return True, {
                    "pair": (a, b),
                    "distance": dist,
                }

        return False, {
            "pair": None,
            "distance": (
                min_distance
                if min_distance != float("inf")
                else None
            ),
        }