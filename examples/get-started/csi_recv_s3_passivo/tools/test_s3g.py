"""Teste de mesa do parser e da análise com linhas sintéticas: pytest test_s3g.py"""
import base64
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from s3g_comum import parse_s3g, xor_ok  # noqa: E402
import s3g_analise  # noqa: E402


def linha(seq, agc=30, fft=2, comp=1000, forc=0, fwi=0, escala=1.0, mac="1a:00:00:00:00:00"):
    v = np.zeros(128, dtype=np.int8)
    v[10:118] = np.clip(np.round(20 * escala), -128, 127)   # amplitude ~ 28*escala
    corpo = (f"S3G,{seq},{seq * 5000},{mac},-50,-95,11,1,0,0,{agc},{fft},1,{comp},{forc},{fwi},128,"
             f"{seq + 1},0,{base64.b64encode(v.tobytes()).decode()}")
    x = 0
    for b in corpo.encode():
        x ^= b
    return f"{corpo}*{x:02X}"


def test_parse_ok_e_corrompida():
    l = linha(7)
    assert xor_ok(l)
    d = parse_s3g(l)
    assert d["seq"] == 7 and d["len"] == 128 and len(d["csi"]) == 128 and d["mac"] == "1a:00:00:00:00:00"
    assert parse_s3g(l.replace("S3G,7,", "S3G,8,")) is None          # XOR errado
    assert parse_s3g(l[:-10] + l[-3:]) is None                         # truncada


def test_compensacao_reduz_cv(tmp_path):
    # ganho alterna; o firmware informa comp que desfaz a escala -> CV compensado ~ 0
    p = tmp_path / "x.s3g.txt"
    with open(p, "w") as f:
        for i in range(3000):
            alto = (i // 50) % 2 == 1
            esc = 1.5 if alto else 1.0
            f.write(f"0.0 {linha(i, agc=40 if alto else 30, comp=int(1000 / esc), escala=esc)}\n")
    qs = s3g_analise.carregar(str(p))["1a:00:00:00:00:00"]
    r = s3g_analise.resumo_mac("1a", qs)
    assert r["ganhos_distintos"] == 2 and r["trocas_de_ganho"] == 59
    assert r["cv_media_comp_auto"] < 0.05 < r["cv_media_cru_auto"]
