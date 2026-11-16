"""Check the mechanical subset of RoboPico C Standard 1.0; no hardware access."""
import argparse
import json
import re
import sys
from pathlib import Path

from scope import ROOT, source_files

NON_CODE = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', re.S)


def mask_non_code(text):
    return NON_CODE.sub(lambda match: re.sub(r"[^\n]", " ", match[0]), text)


def inspect(path):
    text = path.read_text(encoding="utf-8")
    code = mask_non_code(text)
    findings = []

    def note(at, rule, message):
        findings.append({"file": path.relative_to(ROOT).as_posix(),
                         "line": text.count("\n", 0, at) + 1,
                         "rule": rule, "message": message})

    for match in re.finditer(r"\t", text):
        note(match.start(), "3.5.a", "Literal tab; use four-space indentation.")
    for match in re.finditer(r"[^\S\n]+$", text, re.M):
        note(match.start(), "3", "Trailing whitespace.")
    if "@file" not in text[:1600]:
        note(0, "2.2.h", "Missing module documentation.")
    if not text.rstrip().endswith("/* End of file. */"):
        note(max(0, len(text) - 1), "3.3.c", "Missing EOF comment.")
    for match in re.finditer(r"\b(?:signed\s+(?:int\s+)?|int(?:8|16|32|64)_t\s+)\w+\s*:\s*\d+", code):
        note(match.start(), "5.3.a", "Signed bit-field; use explicit unsigned storage and conversion.")
    typedef = re.compile(r"\btypedef\b(?:[^;{}]|\{[^{}]*\})+?\b([A-Za-z_]\w*)\s*;", re.S)
    for match in typedef.finditer(code):
        if not re.fullmatch(r"[a-z][a-z0-9_]*_t", match[1]):
            note(match.start(1), "5.1.a", "Application type must be lowercase and end in _t.")
    if path.suffix == ".h":
        if not re.search(r"^\s*#ifndef\s+\w+", code, re.M):
            note(0, "4.2.b", "Missing include guard.")
    if path.suffix == ".c":
        for match in re.finditer(r'^\s*#include\s+["<][^\n]+\.c[">]', text, re.M):
            note(match.start(), "4.3.f", "Source must not include another C source.")
    return findings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, help="Write the mechanical-check report.")
    args = parser.parse_args()
    files = source_files()
    findings = [item for path in files for item in inspect(path)]
    report = {"standard": "RoboPico C Standard 1.0 (adapted Barr-C:2018)",
              "application_files": len(files), "findings": findings,
              "limitations": "Mechanical subset only; not whole-standard semantic certification. "
                             "Run pinned formatting separately and review conversions, contracts, "
                             "concurrency and all conditional configurations."}
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    for item in findings:
        print(f"{item['file']}:{item['line']}: {item['rule']}: {item['message']}")
    print(f"{'FAIL' if findings else 'PASS'} mechanical C standard subset: {len(files)} files, {len(findings)} findings")
    return bool(findings)


if __name__ == "__main__":
    sys.exit(main())
