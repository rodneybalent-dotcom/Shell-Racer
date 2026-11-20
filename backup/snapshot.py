#!/usr/bin/env python3
"""Publish a scoped Shell Racer backup from the Node-RED host."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

TABS = {
    "36717e86bd974863", "77e102568a434585",
    "a13fbc65dbbbf1d4", "c981f46fb5228101",
}

INLINE_SECRET = re.compile(
    r"-----BEGIN [A-Z ]*PRIVATE KEY-----|https?://[^/\s'\"@]+:[^/\s'\"@]+@"
    r"|(?:bearer|basic)\s+[a-z0-9_+/=-]{16,}"
    r"|(?:password|api[_-]?key|secret|token)\s*[:=]\s*['\"][^'\"\n]{8,}['\"]",
    re.I,
)
LOCAL_DETAILS = re.compile(
    r"/(?:home|Users|Volumes)/[^/\s'\"]+/|\b[\w.-]+\.(?:local|internal|edu\.au)\b"
    r"|\b(?:10\.\d+|172\.(?:1[6-9]|2\d|3[01])|192\.168)\.\d+\.\d+\b"
    r"|\b[\w.+-]+@[\w.-]+\.[a-z]{2,}\b",
    re.I,
)


def public_dashboard(function):
    function = re.sub(
        r"env\.get\('SHELLRACER_AUDIO_DIR'\) \|\| '/[^'\n]*'",
        "env.get('SHELLRACER_AUDIO_DIR')",
        function,
    )
    return re.sub(
        r"\[(?:'[a-z0-9.-]+',)*'localhost'(?:,'[a-z0-9.-]+')*\]\.includes\((hostName|hostname)\)",
        r"['localhost', ...String(env.get('SHELLRACER_ALLOWED_HOSTS') || '').toLowerCase()"
        r".split(',').map(value => value.trim())].includes(\1)",
        function,
    )


def check_public(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if item and re.search(r"password|credential|secret|token|api.?key|authorization", key, re.I):
                raise ValueError("A credential-like field was found. Review it before publishing.")
            check_public(item)
    elif isinstance(value, list):
        for item in value:
            check_public(item)
    elif isinstance(value, str) and (INLINE_SECRET.search(value) or LOCAL_DETAILS.search(value)):
        raise ValueError("Possible private information was found. Review it before publishing.")


def references(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, list):
        for item in value:
            yield from references(item)
    elif isinstance(value, dict):
        for item in value.values():
            yield from references(item)


def export_flows(path):
    raw = json.loads(path.read_text())
    nodes = raw if isinstance(raw, list) else raw["flows"]
    index = {node["id"]: node for node in nodes}
    if not TABS.issubset(index):
        raise ValueError("Expected Shell Racer tabs are missing. Check the configured flows file.")
    wanted = TABS | {node["id"] for node in nodes if node.get("z") in TABS}
    pending = list(wanted)
    while pending:
        source = index[pending.pop()]
        for ref in references(source):
            if ref in wanted or ref not in index:
                continue
            node = index[ref]
            if node.get("z") or node["type"] in {"tab", "subflow"}:
                raise ValueError("Shell Racer references another flow. Review its scope before backing up.")
            wanted.add(ref)
            pending.append(ref)
    result = [node for node in nodes if node["id"] in wanted]
    for node in result:
        if node.get("name") in {"Piper and custom MP3 cues", "Drive arbitration"}:
            node["func"] = public_dashboard(node["func"])
        check_public(node)
    return (json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True)+"\n").encode()


def firmware(source):
    contents = source.read_text()
    values = {
        "WIFI_SSID": "YOUR_WIFI_SSID", "WIFI_PASSWORD": "YOUR_WIFI_PASSWORD",
        "NODE_RED_URL": "http://YOUR_NODE_RED_HOST:1880/shellracer/telemetry",
    }
    for name, placeholder in values.items():
        contents, count = re.subn(
            rf'(const char {name}\[\]\s*=\s*)"(?:\\.|[^"\\])*"',
            lambda match: match[1]+'"'+placeholder+'"', contents,
        )
        if count != 1:
            raise ValueError("Expected firmware setting missing: "+name)
    if LOCAL_DETAILS.search(contents) or INLINE_SECRET.search(contents) or re.search(r"(?:password|secret|token|api[_-]?key)\w*\s*(?:\[\])?\s*=\s*\"(?!YOUR_)[^\"]+\"", contents, re.I):
        raise ValueError("Review possible inline firmware secrets before publishing.")
    return contents.encode()


def put(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_bytes() == data:
        return
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as tmp:
        tmp.write(data)
    os.replace(tmp.name, path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.expanduser().read_text())
    root = Path(config["checkout"]).expanduser().resolve()
    def git(*parts, **kwargs):
        return subprocess.run(["git", "-C", str(root), *parts], check=True, **kwargs)
    git_dir = Path(git("rev-parse", "--absolute-git-dir", capture_output=True, text=True).stdout.strip())
    with (git_dir/"shell-racer-backup.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if git("status","--porcelain",capture_output=True,text=True).stdout.strip():
            raise ValueError("Backup checkout has local edits. Use a dedicated clean checkout.")
        git("pull", "--rebase", "--autostash")
        changes = {"node-red/ShellRacer.flows.json": export_flows(Path(config["flows_file"]).expanduser())}
        for path in config.get("firmware_sources", []):
            src = Path(path).expanduser()
            if src.suffix != ".ino" or src.stem != src.parent.name:
                raise ValueError("Each Arduino sketch must be in a same-named directory.")
            changes[f"firmware/{src.stem}/{src.name}"] = firmware(src)
        tracked = list(changes)
        for path, data in changes.items():
            put(root/path, data)
        git("add", "--", *tracked)
        diff = subprocess.run(["git","-C",str(root),"diff","--cached","--quiet","--",*tracked])
        if diff.returncode==1:
            git("commit","-m","Back up Shell Racer","--",*tracked)
        elif diff.returncode:
            raise RuntimeError("Could not compare staged backup.")
        git("push")


if __name__ == "__main__":
    main()
