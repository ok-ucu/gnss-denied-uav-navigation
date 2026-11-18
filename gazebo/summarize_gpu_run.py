#!/usr/bin/env python3
"""Збирає SUMMARY.md з папки доказів run_linux_gpu.sh (тільки stdlib)."""
import csv, json, os, re, sys

d = sys.argv[1]
def rd(name):
    p = os.path.join(d, name)
    return open(p, errors="replace").read() if os.path.isfile(p) else ""
def js(name):
    p = os.path.join(d, name)
    return json.load(open(p)) if os.path.isfile(p) else None

smi = rd("nvidia-smi_during_run.txt")
gazebo_log = rd("gazebo.log")
env = rd("env.txt")
stream, rec = js("stream.json"), js("recording.json")

sw = re.findall(r"llvmpipe|softpipe|swrast", gazebo_log + rd("renderer_check.txt").split("--- пошук")[0], re.I)
proc_listed = bool(re.search(r"\b(gz|ruby|gz-sim)\b", smi, re.I))
utils = []
p = os.path.join(d, "gpu_util.csv")
if os.path.isfile(p):
    for row in list(csv.reader(open(p)))[1:]:
        m = re.search(r"(\d+)\s*%", row[1]) if len(row) > 1 else None
        if m: utils.append(int(m.group(1)))
max_u = max(utils) if utils else None
mean_u = sum(utils) / len(utils) if utils else None
sw_env = "LIBGL_ALWAYS_SOFTWARE=<unset>" in env

gpu_ok = (not sw) and sw_env and (proc_listed or (max_u is not None and max_u >= 5))
lines = ["# Linux GPU validation — підсумок", ""]
lines.append(f"**GPU-рендер:** {'ПІДТВЕРДЖЕНО (непрямі докази, див. нижче)' if gpu_ok else 'НЕ ПІДТВЕРДЖЕНО'}")
lines += ["", f"- LIBGL_ALWAYS_SOFTWARE у контейнері: {'не заданий' if sw_env else 'ЗАДАНИЙ'}",
          f"- software-рендерер у логах (llvmpipe/softpipe/swrast): {'ТАК: ' + ', '.join(set(sw)) if sw else 'ні'}",
          f"- процес gz/ruby у nvidia-smi під час роботи: {'так' if proc_listed else 'ні'}",
          f"- завантаження GPU, %: макс {max_u}, середнє {mean_u:.1f}" if utils else "- завантаження GPU: немає даних (gpu_util.csv порожній)"]
lines += ["", f"**Потік (live):** {stream['verdict'] if stream else 'НЕМАЄ stream.json'}"]
if stream:
    lines.append(f"- RTF: {stream.get('rtf')}")
    for c in stream["checks"]:
        lines.append(f"- {'PASS' if c['ok'] else 'FAIL'}: {c['name']} — {c['detail']}")
lines += ["", f"**Запис:** {rec['verdict'] if rec else 'НЕМАЄ recording.json'}"]
if rec:
    for c in rec["checks"]:
        lines.append(f"- {'PASS' if c['ok'] else 'FAIL'}: {c['name']} — {c['detail']}")
overall = gpu_ok and stream and stream["verdict"] == "PASS" and rec and rec["verdict"] == "PASS"
lines += ["", f"## ЗАГАЛЬНИЙ ВЕРДИКТ: {'PASS' if overall else 'FAIL / потребує розбору'}", "",
          "Примітка: GPU-використання встановлено за nvidia-smi і відсутністю software-рендерера в логах; "
          "це непрямі, але стандартні докази."]
open(os.path.join(d, "SUMMARY.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
