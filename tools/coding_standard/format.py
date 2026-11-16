"""Apply the pinned clang-format 18.1.8 application style, or check it."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

from scope import ROOT, source_files


def find_formatter():
    local = ROOT.parent / ".buddy2-tools/formatter/clang_format/data/bin/clang-format.exe"
    return os.environ.get("CLANG_FORMAT") or (str(local) if local.is_file() else
                                              shutil.which("clang-format"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    tool = find_formatter()
    if not tool:
        parser.error("Install clang-format 18.1.8 or set CLANG_FORMAT to its executable.")
    version = subprocess.check_output([tool, "--version"], text=True)
    if "18.1.8" not in version:
        parser.error("Pinned formatter version is 18.1.8: " + version.strip())
    files = source_files()
    for start in range(0, len(files), 25):
        subprocess.run([tool, *(('--dry-run', '--Werror') if args.check else ('-i',)),
                        *map(str, files[start:start + 25])], check=True, cwd=ROOT)
    print(f"PASS clang-format 18.1.8 {'check' if args.check else 'applied'}: {len(files)} application files")


if __name__ == "__main__":
    main()
