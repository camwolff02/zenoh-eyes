# tools/gen_flatbuffers.py
# Generate C++ headers from schemas/*.fbs into ./include using flatc.
# Only regenerates when the output header is missing or older than the schema.
from pathlib import Path
import shutil, subprocess

Import("env")  # Provided by PlatformIO

def log(msg): print(f"[gen_flatbuffers] {msg}")

def generate(env):
    proj = Path(env["PROJECT_DIR"])
    schemas_dir = proj / "schemas"
    out_dir = proj / "include"

    # Resolve flatc
    custom_flatc = None
    try:
        # env.GetProjectOptions() may not exist in older PIO; guard it
        if "custom_flatc" in env.GetProjectOptions():
            custom_flatc = env.GetProjectOption("custom_flatc")
    except Exception:
        pass
    flatc = custom_flatc or shutil.which("flatc") or shutil.which("flatc.exe")
    if not flatc:
        log("flatc not found; skipping FlatBuffers codegen. (Set 'custom_flatc' in platformio.ini or add to PATH.)")
        return  # do not abort the whole build

    if not schemas_dir.exists():
        log(f"No schemas/ directory at {schemas_dir}; skipping.")
        return

    fbs_files = sorted(schemas_dir.rglob("*.fbs"))
    if not fbs_files:
        log("No .fbs files found under schemas/; skipping.")
        return

    out_dir.mkdir(parents=True, exist_ok=True)

    generated_any = False
    for fbs in fbs_files:
        header_name = fbs.stem + "_generated.h"
        header_path = out_dir / header_name
        need_build = (not header_path.exists()) or (fbs.stat().st_mtime > header_path.stat().st_mtime)

        if not need_build:
            log(f"Up to date: {header_path.name}")
            continue

        cmd = [flatc, "--cpp", "-o", str(out_dir), "-I", str(schemas_dir), str(fbs)]
        log("Running: " + " ".join(cmd))
        try:
            subprocess.check_call(cmd, cwd=str(proj))
            generated_any = True
            if header_path.exists():
                log(f"Generated: {header_path.name}")
            else:
                log(f"Warning: expected {header_path.name} not found after flatc (schema: {fbs.name}).")
        except subprocess.CalledProcessError as e:
            # Hard error on this schema, but don't raise SystemExit which could mask previous logs.
            raise  # let PlatformIO show the failure

    if not generated_any:
        log("All FlatBuffers headers are up to date.")

# Run without exiting so previous/next scripts still run
generate(env)
