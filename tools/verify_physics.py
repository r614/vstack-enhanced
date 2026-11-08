"""Check every signature in lanvp/fix_physics.cpp against a LANoire.exe.

Each constant redirect must match exactly once in .text and its operand must
point at the expected original value; each code patch must match exactly once.

usage: uv run --with pefile python tools/verify_physics.py [LANoire.exe path]
"""
import os
import re
import struct
import sys

import pefile

DEFAULT_EXE = r"C:\Program Files (x86)\Steam\steamapps\common\L.A.Noire\LANoire.exe"
EXE = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_EXE
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lanvp", "fix_physics.cpp")

pe = pefile.PE(EXE, fast_load=True)
img = pe.get_memory_mapped_image()
base = pe.OPTIONAL_HEADER.ImageBase
text = next(s for s in pe.sections if s.Name.rstrip(b"\0") == b".text")
t0, t1 = text.VirtualAddress, text.VirtualAddress + text.Misc_VirtualSize

src = open(SRC, encoding="utf-8").read()

patterns = {}
for m in re.finditer(r"static const int (\w+)\[\]\s*=\s*\{(.*?)\};", src, re.S):
    body = re.sub(r"//.*", "", m.group(2))
    patterns[m.group(1)] = [None if b.strip() == "ANY" else int(b, 16) for b in body.split(",") if b.strip()]


def find(pattern):
    rx = b"".join(b"." if b is None else re.escape(bytes([b])) for b in pattern)
    return [mm.start() for mm in re.finditer(rx, img[t0:t1], re.S)]


CONST_RE = re.compile(
    r'\{\s*L"([^"]+)",\s*PATTERN\((\w+)\),\s*(\d+),\s*(true|false),\s*([-0-9.e]+),\s*(SCALE_\w+)(?:,\s*[-0-9.e]+)?\s*\}'
)
CODE_RE = re.compile(r'\{\s*L"([^"]+)",\s*PATTERN\((\w+)\),\s*(\d+),\s*(\d+)\s*\}')

checked = 0
ok = True

for m in CONST_RE.finditer(src):
    name, pat, off = m.group(1), m.group(2), int(m.group(3))
    is_double, expected, kind = m.group(4) == "true", float(m.group(5)), m.group(6)
    checked += 1
    hits = find(patterns[pat])
    if len(hits) != 1:
        print(f"FAIL {name}: {len(hits)} matches")
        ok = False
        continue
    va = base + t0 + hits[0]
    addr = struct.unpack_from("<I", img, t0 + hits[0] + off)[0]
    value = struct.unpack_from("<d" if is_double else "<f", img, addr - base)[0]
    good = abs(value - expected) <= 1e-6 * abs(expected)
    ok &= good
    print(f"{'ok  ' if good else 'FAIL'} {name:30s} @ {va:#x}  [{addr:#x}] = {value!r} "
          f"({'double' if is_double else 'float'}, {kind})")

for m in CODE_RE.finditer(src):
    name, pat, off, length = m.group(1), m.group(2), int(m.group(3)), int(m.group(4))
    checked += 1
    hits = find(patterns[pat])
    if len(hits) != 1:
        print(f"FAIL {name}: {len(hits)} matches")
        ok = False
        continue
    va = base + t0 + hits[0]
    old = img[t0 + hits[0] + off:t0 + hits[0] + off + length].hex(" ")
    print(f"ok   {name:30s} @ {va:#x}  NOP {length} bytes at {va + off:#x} ({old})")

rows = len(re.findall(r'\{\s*L"[^"]+",\s*PATTERN\(', src))
if checked == 0 or checked != rows:
    print(f"FAIL parsed {checked} of {rows} table rows")
    ok = False

print(f"{checked} entries: " + ("ALL OK" if ok else "FAILURES"))
sys.exit(0 if ok else 1)
