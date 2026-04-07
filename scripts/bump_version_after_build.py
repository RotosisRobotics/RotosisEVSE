import json
import re
from pathlib import Path

Import("env")


_SEMVER_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)$")


def _bump_patch_version(version):
    match = _SEMVER_RE.match(version.strip())
    if not match:
        raise ValueError(f"Unsupported version format: {version!r}")

    major, minor, patch = (int(part) for part in match.groups())
    return f"{major}.{minor}.{patch + 1}"


def _bump_version_after_success(source, target, env):
    project_dir = Path(env["PROJECT_DIR"])
    version_path = project_dir / "version.json"
    if not version_path.exists():
        print("[version-bump] Skip: version.json not found")
        return

    data = json.loads(version_path.read_text(encoding="utf-8-sig"))
    current_version = str(data.get("version", "")).strip()
    if not current_version:
        print("[version-bump] Skip: version field is empty")
        return

    next_version = _bump_patch_version(current_version)
    data["version"] = next_version
    version_path.write_text(
        json.dumps(data, ensure_ascii=False, indent=4) + "\n",
        encoding="utf-8",
    )
    print(
        f"[version-bump] Build used {current_version}; version.json advanced to {next_version}"
    )


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", _bump_version_after_success)
