#!/usr/bin/env python3
"""
check_xml.py : catch malformed XML in URDF/Xacro before colcon does.

WHY THIS EXISTS
===============
Xacro's parse errors report a line and column in the FULLY EXPANDED output,
not in the file you edited, and the expansion of a six leg macro bears little
resemblance to the source. A trivial mistake therefore costs far more time to
locate than it should.

The specific trap this was written for: XML forbids the two character
sequence "- -" (without the space) ANYWHERE inside a comment body, because
that sequence is reserved as the comment terminator prefix. Using it as a
decorative separator, which looks completely natural, silently makes the file
invalid. The error you get is:

    XML parsing error: not well-formed (invalid token): line 3, column 24

...which does not mention comments at all.

This script reports the offending file, line and column directly.

USAGE
=====
    python3 tools/check_xml.py src/hexapod_description/urdf
    python3 tools/check_xml.py src            # recurses

Run it before every build. It takes milliseconds and it is the cheapest
possible insurance against a class of bug that is disproportionately annoying
to diagnose.
"""

import pathlib
import re
import sys
import xml.etree.ElementTree as ET

BAD_SEQ = "-" + "-"                       # written this way to stay legal here
COMMENT_RE = re.compile(r"<!--(.*?)-->", re.DOTALL)
EXTENSIONS = {".xacro", ".urdf", ".xml", ".sdf", ".world"}


def check_file(path: pathlib.Path) -> list[str]:
    errors: list[str] = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return [f"{path}: cannot read ({exc})"]

    # ---- 1. illegal double hyphen inside comment bodies ------------------
    for match in COMMENT_RE.finditer(text):
        body = match.group(1)
        if BAD_SEQ in body:
            offset = match.start(1) + body.index(BAD_SEQ)
            line = text.count("\n", 0, offset) + 1
            col = offset - text.rfind("\n", 0, offset)
            errors.append(
                f"{path}:{line}:{col}: illegal '{BAD_SEQ}' inside an XML "
                f"comment (reserved as the comment terminator prefix)"
            )

    # ---- 2. unterminated comment ----------------------------------------
    opens = text.count("<!" + BAD_SEQ)
    closes = text.count(BAD_SEQ + ">")
    if opens != closes:
        errors.append(
            f"{path}: {opens} comment openers vs {closes} closers "
            "(unterminated or nested comment)"
        )

    # ---- 3. does it actually parse? -------------------------------------
    # Xacro files use ${...} inside ATTRIBUTE VALUES only, which is valid XML,
    # so a raw parse is a legitimate structural test.
    try:
        ET.fromstring(text)
    except ET.ParseError as exc:
        errors.append(f"{path}: XML parse error: {exc}")

    return errors


def main() -> int:
    target = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")

    if target.is_file():
        files = [target]
    else:
        files = sorted(
            p for p in target.rglob("*")
            if p.is_file() and p.suffix in EXTENSIONS
        )

    if not files:
        print(f"No XML-ish files found under {target}")
        return 1

    all_errors: list[str] = []
    for f in files:
        all_errors.extend(check_file(f))

    if all_errors:
        print(f"FAILED : {len(all_errors)} problem(s) in {len(files)} file(s)\n")
        for e in all_errors:
            print(f"  {e}")
        return 1

    print(f"OK : {len(files)} file(s) are well formed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
