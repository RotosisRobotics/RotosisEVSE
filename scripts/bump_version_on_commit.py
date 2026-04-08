import json
import re
import subprocess
import sys
import time
from pathlib import Path


SEMVER_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)$")


def bump_patch(version: str) -> str:
    match = SEMVER_RE.fullmatch(version.strip())
    if not match:
        raise ValueError(f"Unsupported version format: {version!r}")

    major, minor, patch = (int(part) for part in match.groups())
    return f"{major}.{minor}.{patch + 1}"


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    version_path = repo_root / "version.json"
    if not version_path.exists():
        print("[version-hook] Skip: version.json not found")
        return 0

    data = json.loads(version_path.read_text(encoding="utf-8-sig"))
    current_version = str(data.get("version", "")).strip()
    if not current_version:
        print("[version-hook] Skip: version field is empty")
        return 0

    next_version = bump_patch(current_version)
    data["version"] = next_version
    version_path.write_text(
        json.dumps(data, ensure_ascii=False, indent=4) + "\n",
        encoding="utf-8",
    )

    for attempt in range(10):
        add_result = subprocess.run(
            ["git", "add", str(version_path)],
            cwd=repo_root,
            capture_output=True,
            text=True,
        )
        if add_result.returncode == 0:
            break
        if "index.lock" not in (add_result.stderr or ""):
            add_result.check_returncode()
        time.sleep(0.2)
    else:
        raise RuntimeError("git add failed repeatedly because index.lock stayed busy")

    print(f"[version-hook] version.json {current_version} -> {next_version}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[version-hook] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
