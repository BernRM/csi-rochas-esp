"""Leitura das linhas do firmware csi_recv_s3_passivo (formato S3G). Só stdlib."""
import base64

COLS = ["seq", "t_us", "mac", "rssi", "noise_floor", "rate", "sig_mode", "mcs", "cwb",
        "agc", "fft", "gain_status", "comp_x1000", "forcado", "first_word_invalid", "len",
        "rx_total", "fila_cheia", "csi_base64"]
INTS = set(COLS) - {"mac", "csi_base64"}


def xor_ok(linha: str) -> bool:
    """Confere o sufixo *XX (XOR de todos os bytes antes do '*')."""
    if len(linha) < 4 or linha[-3] != "*":
        return False
    x = 0
    for b in linha[:-3].encode("ascii", "replace"):
        x ^= b
    try:
        return x == int(linha[-2:], 16)
    except ValueError:
        return False


def parse_s3g(linha: str):
    """Devolve dict de uma linha S3G válida, ou None. `csi` vem como bytes crus."""
    linha = linha.strip()
    if not linha.startswith("S3G,") or not xor_ok(linha):
        return None
    campos = linha[4:-3].split(",")
    if len(campos) != len(COLS):
        return None
    d = dict(zip(COLS, campos))
    try:
        for k in INTS:
            d[k] = int(d[k])
        csi = base64.b64decode(d.pop("csi_base64"), validate=True)
    except (ValueError, base64.binascii.Error):
        return None
    if len(csi) != d["len"]:
        return None
    d["csi"] = csi
    return d


def amostras_iq(csi: bytes, first_word_invalid: int):
    """Bytes do CSI (int8, pares imag,real) -> lista de (real, imag).
    Se first_word_invalid, os 4 primeiros bytes (2 subportadoras) viram None."""
    v = [b - 256 if b > 127 else b for b in csi]
    out = []
    for i in range(0, len(v) - 1, 2):
        if first_word_invalid and i < 4:
            out.append(None)
        else:
            out.append((v[i + 1], v[i]))
    return out
