#!/usr/bin/env python3
"""Bundle src/core.js + src/mock.js + i18n/*.json + src/ui.js into dist/servo-id-tool.v<N>.js (no minifier, no deps)."""
import json, glob, os, sys
H = os.path.dirname(os.path.abspath(__file__))
VER = sys.argv[1] if len(sys.argv) > 1 else "1"
i18n = {os.path.basename(p)[:-5]: json.load(open(p)) for p in sorted(glob.glob(f"{H}/i18n/*.json"))}
en = set(i18n["en"])
for k, d in i18n.items():   # every language must have exactly the English keys, and keep the {placeholders}
    assert set(d) == en, (k, sorted(en ^ set(d)))
    for key in en:
        import re
        assert sorted(re.findall(r"\{\w+\}", d[key])) == sorted(re.findall(r"\{\w+\}", i18n["en"][key])), (k, key)
out = ["/*! ArmDojo Servo ID tool v%s — https://armdojo.com/pages/servo-id-tool */" % VER]
for f in ("core.js", "mock.js"):
    out.append(open(f"{H}/src/{f}").read())
out.append("window.ArmDojoServoI18n=" + json.dumps(i18n, ensure_ascii=False, separators=(",", ":")) + ";")
out.append(open(f"{H}/src/ui.js").read())
dst = f"{H}/dist/servo-id-tool.v{VER}.js"
open(dst, "w").write("\n".join(out))
print(dst, os.path.getsize(dst), "bytes,", len(i18n), "languages")
