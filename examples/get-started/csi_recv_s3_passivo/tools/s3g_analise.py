#!/usr/bin/env python3
"""Passo 3 da fase 2 (exploratório): o ganho do S3 varia? A compensação reduz a variância?

Entrada: <prefixo>.s3g.txt gerado por s3g_captura.py. Precisa de numpy.

Para cada MAC com pelo menos --min-quadros quadros:
  - distribuição de (agc, fft) e número de trocas de ganho;
  - CV da amplitude média por quadro: cru, compensado (cru * comp) e só nos quadros travados;
  - mediana do CV por subportadora (cru e compensado);
  - salto |Δ amplitude média| nos quadros em que o ganho mudou vs. nos que não mudou.
"""
import argparse
import os
import sys
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from s3g_comum import parse_s3g  # noqa: E402


def carregar(path):
    por_mac = defaultdict(list)
    for linha in open(path, encoding="ascii", errors="replace"):
        partes = linha.split(" ", 1)
        if len(partes) != 2:
            continue
        d = parse_s3g(partes[1])
        if d:
            por_mac[d["mac"]].append(d)
    return por_mac


def amplitudes(quadros):
    """Matriz [quadro, subportadora] de amplitude crua (NaN onde inválido)."""
    n = max(q["len"] for q in quadros) // 2
    A = np.full((len(quadros), n), np.nan)
    for i, q in enumerate(quadros):
        v = np.frombuffer(q["csi"], dtype=np.int8).astype(float)
        im, re = v[0::2], v[1::2]
        a = np.hypot(re, im)
        if q["first_word_invalid"]:
            a[:2] = np.nan
        A[i, :len(a)] = a
    return A


def cv(x):
    x = x[np.isfinite(x)]
    return float(np.std(x) / np.mean(x)) if len(x) > 1 and np.mean(x) > 0 else float("nan")


def resumo_mac(mac, qs):
    A = amplitudes(qs)
    ativo = np.nanmean(A, axis=0) > 1.0          # descarta subportadoras nulas/guarda
    A = A[:, ativo]
    comp = np.array([q["comp_x1000"] / 1000.0 if q["comp_x1000"] > 0 else np.nan for q in qs])
    forc = np.array([q["forcado"] for q in qs]) == 1
    ganho = [(q["agc"], q["fft"]) for q in qs]
    trocas = np.array([False] + [ganho[i] != ganho[i - 1] for i in range(1, len(ganho))])
    m_cru = np.nanmean(A, axis=1)
    m_comp = m_cru * comp
    auto = ~forc & np.isfinite(comp)
    dm = np.abs(np.diff(m_cru, prepend=np.nan))
    r = {
        "mac": mac, "quadros": len(qs), "subportadoras_ativas": int(ativo.sum()),
        "rssi_mediano": float(np.median([q["rssi"] for q in qs])),
        "ganhos_distintos": len(set(ganho)), "trocas_de_ganho": int(trocas.sum()),
        "top_ganhos": Counter(ganho).most_common(4),
        "cv_media_cru_auto": cv(m_cru[auto]), "cv_media_comp_auto": cv(m_comp[auto]),
        "cv_media_travado": cv(m_cru[forc]) if forc.any() else float("nan"),
        "cv_subp_cru_auto": float(np.nanmedian([cv(A[auto, j]) for j in range(A.shape[1])])) if auto.any() else float("nan"),
        "cv_subp_comp_auto": float(np.nanmedian([cv(A[auto, j] * comp[auto]) for j in range(A.shape[1])])) if auto.any() else float("nan"),
        "salto_com_troca": float(np.nanmedian(dm[trocas])) if trocas.any() else float("nan"),
        "salto_sem_troca": float(np.nanmedian(dm[~trocas])),
        "quadros_travados": int(forc.sum()),
    }
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("captura", help="<prefixo>.s3g.txt")
    ap.add_argument("--min-quadros", type=int, default=2000)
    ap.add_argument("--saida-md", default=None)
    a = ap.parse_args()

    por_mac = carregar(a.captura)
    linhas = ["| MAC | quadros | RSSI med | ganhos distintos | trocas | CV média cru (auto) | "
              "CV média comp (auto) | CV média travado | CV subp cru | CV subp comp | "
              "salto c/ troca | salto s/ troca |", "|" + "---|" * 12]
    for mac, qs in sorted(por_mac.items(), key=lambda kv: -len(kv[1])):
        if len(qs) < a.min_quadros:
            continue
        r = resumo_mac(mac, qs)
        linhas.append("| {mac} | {quadros} | {rssi_mediano:.0f} | {ganhos_distintos} | {trocas_de_ganho} | "
                      "{cv_media_cru_auto:.4f} | {cv_media_comp_auto:.4f} | {cv_media_travado:.4f} | "
                      "{cv_subp_cru_auto:.4f} | {cv_subp_comp_auto:.4f} | {salto_com_troca:.2f} | "
                      "{salto_sem_troca:.2f} |".format(**r))
        linhas.append(f"|  ↳ top (agc,fft): {r['top_ganhos']}; travados: {r['quadros_travados']} |" + " |" * 11)
    total = sum(len(v) for v in por_mac.values())
    texto = f"Quadros válidos: {total}; MACs: {len(por_mac)}\n\n" + "\n".join(linhas) + "\n"
    print(texto)
    if a.saida_md:
        with open(a.saida_md, "w") as f:
            f.write(texto)


if __name__ == "__main__":
    main()
