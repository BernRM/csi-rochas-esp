#!/usr/bin/env python3
"""Captura do csi_recv_s3_passivo pela serial (só pyserial + stdlib).

Grava as linhas S3G*/S3G_* cruas, cada uma prefixada pelo horário do PC (epoch em s), e um
.meta.json com contadores. Opcionalmente envia 'F' (trava o ganho) depois de N segundos.

Exemplo:
  python s3g_captura.py --porta /dev/ttyACM0 --duracao 1200 --forcar-apos 600 --saida captura_20261106
"""
import argparse
import hashlib
import json
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from s3g_comum import parse_s3g, xor_ok  # noqa: E402


def sha256(path):
    if not path or not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--porta", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--duracao", type=float, required=True, help="segundos")
    ap.add_argument("--forcar-apos", type=float, default=None, help="envia 'F' depois de N s")
    ap.add_argument("--saida", required=True, help="prefixo dos arquivos de saída")
    ap.add_argument("--bin", default=None, help="caminho do .bin gravado (para o sha256 no meta)")
    a = ap.parse_args()

    txt, meta_path = a.saida + ".s3g.txt", a.saida + ".meta.json"
    if os.path.exists(txt):
        sys.exit(f"já existe: {txt} (não sobrescrevo)")

    cont = {"linhas_s3g_validas": 0, "linhas_s3g_invalidas": 0, "outras_linhas": 0,
            "buracos_seq": 0, "quadros_perdidos_seq": 0}
    eventos, hdr, ultimo_stat, seq_ant = [], None, None, None
    meta = {"porta": a.porta, "baud": a.baud, "duracao_s": a.duracao, "forcar_apos_s": a.forcar_apos,
            "bin": a.bin, "bin_sha256": sha256(a.bin), "inicio_epoch": time.time()}

    s = serial.Serial(a.porta, a.baud, timeout=0.5)
    s.reset_input_buffer()
    t0, enviado_f = time.time(), False
    with open(txt, "w", buffering=1 << 16) as out:
        buf = b""
        while time.time() - t0 < a.duracao:
            if a.forcar_apos is not None and not enviado_f and time.time() - t0 >= a.forcar_apos:
                s.write(b"F")
                enviado_f = True
                eventos.append({"pc_epoch": time.time(), "comando": "F"})
            buf += s.read(s.in_waiting or 1)
            *linhas, buf = buf.split(b"\n")
            agora = time.time()
            for raw in linhas:
                l = raw.decode("ascii", "replace").strip()
                if l.startswith("S3G,"):
                    d = parse_s3g(l)
                    if d is None:
                        cont["linhas_s3g_invalidas"] += 1
                        continue
                    cont["linhas_s3g_validas"] += 1
                    if seq_ant is not None and d["seq"] != (seq_ant + 1) & 0xffffffff:
                        cont["buracos_seq"] += 1
                        cont["quadros_perdidos_seq"] += (d["seq"] - seq_ant - 1) & 0xffffffff
                    seq_ant = d["seq"]
                elif l.startswith("S3G_") and xor_ok(l):
                    if l.startswith("S3G_HDR"):
                        hdr = l
                    elif l.startswith("S3G_EVT"):
                        eventos.append({"pc_epoch": agora, "linha": l})
                    elif l.startswith("S3G_STAT"):
                        ultimo_stat = l
                else:
                    cont["outras_linhas"] += 1
                    continue
                out.write(f"{agora:.6f} {l}\n")
    s.close()
    meta.update({"fim_epoch": time.time(), "contadores": cont, "eventos": eventos,
                 "hdr": hdr, "ultimo_stat": ultimo_stat, "arquivo": os.path.basename(txt),
                 "arquivo_sha256": sha256(txt)})
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(json.dumps(cont, ensure_ascii=False))
    print(f"ultimo_stat: {ultimo_stat}")


if __name__ == "__main__":
    main()
