from __future__ import annotations

import hashlib
import importlib.metadata
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

from amr_lite.config import ROOT


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _combined_hash(paths: Iterable[Path], base: Path) -> tuple[str, list[dict[str, str | int]]]:
    digest = hashlib.sha256()
    files = []
    for path in sorted(paths):
        relative = path.relative_to(base).as_posix()
        file_hash = sha256_file(path)
        size = path.stat().st_size
        digest.update(relative.encode("utf-8"))
        digest.update(file_hash.encode("ascii"))
        files.append({"path": relative, "sha256": file_hash, "bytes": size})
    return digest.hexdigest(), files


def source_snapshot() -> dict:
    patterns = ("amr_lite/**/*.py", "configs/*.yaml", "tests/**/*.py", "docs/*.md",
                "README.md", "pyproject.toml")
    paths = {path for pattern in patterns for path in ROOT.glob(pattern) if path.is_file()}
    combined, files = _combined_hash(paths, ROOT)
    return {"sha256": combined, "files": files}


def dataset_snapshot(path: str | Path, version: str) -> dict:
    target = Path(path)
    # Training consumes only direct chunks; nested DAgger data is a separate dataset.
    paths = [item for item in target.glob("*.npz") if item.is_file()]
    combined, files = _combined_hash(paths, target)
    return {"version": version, "root": str(target.resolve()), "sha256": combined, "files": files}


def _run_git(*args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=ROOT.parent,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else f"ERROR: {result.stderr.strip()}"


def git_snapshot() -> dict:
    status = _run_git("status", "--porcelain", "--untracked-files=normal", "--", "amr_lite_spike")
    return {
        "commit": _run_git("rev-parse", "HEAD"),
        "branch": _run_git("branch", "--show-current"),
        "dirty": bool(status),
        "status": status.splitlines(),
    }


def dependency_snapshot() -> dict[str, str]:
    versions = {}
    for distribution in ("numpy", "PyYAML", "torch", "matplotlib", "pytest",
                         "gymnasium", "stable-baselines3"):
        try:
            versions[distribution] = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            versions[distribution] = "not-installed"
    return versions


def build_manifest(*, experiment: dict, dataset: dict, evaluation: dict,
                   checkpoints: Iterable[str | Path], command: list[str] | None = None) -> dict:
    checkpoint_rows = []
    for checkpoint in checkpoints:
        path = Path(checkpoint)
        if path.exists():
            checkpoint_rows.append({
                "path": str(path.resolve()),
                "sha256": sha256_file(path),
                "bytes": path.stat().st_size,
            })
    return {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "experiment": experiment,
        "git": git_snapshot(),
        "source": source_snapshot(),
        "dataset": dataset,
        "evaluation": evaluation,
        "checkpoints": checkpoint_rows,
        "runtime": {
            "python": sys.version,
            "python_executable": sys.executable,
            "platform": platform.platform(),
            "dependencies": dependency_snapshot(),
            "command": command if command is not None else sys.argv,
        },
    }


def write_manifest(path: str | Path, manifest: dict) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    return target
