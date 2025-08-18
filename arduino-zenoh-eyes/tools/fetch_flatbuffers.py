# tools/fetch_flatbuffers.py
# Keep FlatBuffers runtime headers in sync with `flatc --version`.
# Installs into: lib/flatbuffers-headeronly/include/flatbuffers/
from pathlib import Path
import re, os, shutil, subprocess, tempfile, zipfile

Import("env")  # Provided by PlatformIO

def log(msg): print(f"[fetch_flatbuffers] {msg}")

def find_flatc():
    # Allow explicit override via platformio.ini
    try:
        if "custom_flatc" in env.GetProjectOptions():
            p = env.GetProjectOption("custom_flatc")
            if p: return Path(p)
    except Exception:
        pass
    for name in ("flatc", "flatc.exe"):
        p = shutil.which(name)
        if p: return Path(p)
    return None

def detect_flatc_version(flatc_path: Path) -> str:
    out = subprocess.check_output([str(flatc_path), "--version"], text=True).strip()
    # Matches "flatc version 24.3.25" or similar
    m = re.search(r"(\d+)\.(\d+)\.(\d+)", out)
    if not m:
        raise RuntimeError(f"Could not parse flatc version from: {out!r}")
    return ".".join(m.groups())

def parse_runtime_version(header_path: Path) -> str | None:
    try:
        t = header_path.read_text(encoding="utf-8", errors="ignore")
        M = re.search(r"#define\s+FLATBUFFERS_VERSION_MAJOR\s+(\d+)", t)
        m = re.search(r"#define\s+FLATBUFFERS_VERSION_MINOR\s+(\d+)", t)
        r = re.search(r"#define\s+FLATBUFFERS_VERSION_REVISION\s+(\d+)", t)
        if M and m and r:
            return f"{M.group(1)}.{m.group(1)}.{r.group(1)}"
    except Exception:
        pass
    return None

def download(url: str, out_zip: Path):
    # Prefer curl (macOS trust store), fallback to urllib with system CA / certifi if present
    curl = shutil.which("curl")
    if curl:
        log("Downloading with curl …")
        subprocess.check_call([curl, "-fL", "--retry", "3", "-o", str(out_zip), url])
        return
    import ssl, urllib.request
    ctx = ssl.create_default_context()
    try:
        import certifi  # type: ignore
        ctx = ssl.create_default_context(cafile=certifi.where())
    except Exception:
        pass
    log("Downloading with Python urllib …")
    with urllib.request.urlopen(url, context=ctx) as r, open(out_zip, "wb") as f:
        shutil.copyfileobj(r, f)

def extract_headers(zip_file: Path, dest_root: Path):
    with zipfile.ZipFile(zip_file) as zf:
        if dest_root.exists():
            shutil.rmtree(dest_root)
        extracted = 0
        for member in zf.namelist():
            m = member.replace("\\", "/")
            if "/include/flatbuffers/" in m and not m.endswith("/"):
                rel = m.split("include/flatbuffers/", 1)[1]
                out = dest_root / rel
                out.parent.mkdir(parents=True, exist_ok=True)
                with zf.open(member) as src, open(out, "wb") as dst:
                    shutil.copyfileobj(src, dst)
                extracted += 1
    if not (dest_root / "flatbuffers.h").exists():
        raise RuntimeError("Extraction finished but flatbuffers.h not found")
    log(f"Installed runtime headers to {dest_root} ({extracted} files)")

def main():
    proj = Path(env["PROJECT_DIR"])
    dest_root = proj / "lib" / "flatbuffers-headeronly" / "include" / "flatbuffers"
    runtime_h = dest_root / "flatbuffers.h"

    flatc_path = find_flatc()
    if not flatc_path:
        log("flatc not found; skipping runtime sync. (Set 'custom_flatc' or add flatc to PATH.)")
        return

    target = detect_flatc_version(flatc_path)
    log(f"flatc version detected: {target}")

    current = parse_runtime_version(runtime_h) if runtime_h.exists() else None
    if current == target:
        log(f"Runtime v{current} already matches — nothing to do.")
        return

    if current:
        log(f"Updating runtime: v{current} → v{target}")

    url = f"https://github.com/google/flatbuffers/archive/refs/tags/v{target}.zip"
    tmpdir = Path(tempfile.mkdtemp(prefix="pio_flatbuffers_"))
    try:
        zip_path = tmpdir / "flatbuffers.zip"
        log(f"Fetching: {url}")
        download(url, zip_path)
        extract_headers(zip_path, dest_root)
        # Ensure minimal library.json so PIO treats this as a local lib
        lib_dir = dest_root.parent.parent
        lj = lib_dir / "library.json"
        if not lj.exists():
            lj.write_text('{"name":"flatbuffers-headeronly","version":"' + target + '"}\n', encoding="utf-8")
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

# Run without exiting; let following extra_scripts run
main()
