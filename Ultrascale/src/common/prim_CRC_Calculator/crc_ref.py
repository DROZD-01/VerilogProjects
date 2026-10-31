#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
crc_ref.py -- Independent reference model + expected-vector generator for
prim_CRC_Calculator (parametric CRC, Sarwate table-folding).

Verification strategy (three independent layers):

  1. crc_bytes()       : classic bit-serial CRC over a byte stream with the
                         CRC-catalog (reveng / "Rocksoft") conventions.
                         Validated against the published check values of the
                         CRC catalogue for the message "123456789".

  2. crc_beats()       : bit-serial model of the module's *per-beat* semantics
                         (byte-lane input reflection, byte-lane reversal,
                         MSB-first absorption of DATA_WIDTH-bit beats).
                         No tables involved. Cross-checked against (1) for
                         every byte-aligned beat width / packing.

  3. crc_beats_table() : Sarwate table-folding model that mirrors the RTL
                         exactly (CHUNK_W = min(DATA_WIDTH, CRC_WIDTH, 8),
                         full chunks + tail chunk, one lookup per chunk).
                         Fuzzed against (2) over thousands of random
                         parameter combinations.

The SystemVerilog testbench is then fed with expected values emitted from
(3) into crc_expected.svh.

Usage:
    python3 crc_ref.py            # run all self-tests, emit crc_expected.svh
"""

import argparse
import random
import sys

# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------

def reflect(x, w):
    """Bit-reverse the low w bits of x."""
    r = 0
    for i in range(w):
        if (x >> i) & 1:
            r |= 1 << (w - 1 - i)
    return r


def reflect_bytes(data):
    return bytes(reflect(b, 8) for b in data)

# ----------------------------------------------------------------------------
# 1. classic byte-stream reference ("catalog" semantics)
# ----------------------------------------------------------------------------

def crc_bytes(data, width, poly, init, refin, refout, xorout):
    """CRC over a byte stream, MSB-first register, per-byte input reflection.

    This is the classic catalog/reveng convention:
      - the register runs in non-reflected (MSB-first) orientation,
      - if refin, every byte is bit-reflected before entering,
      - if refout, the register is bit-reflected at the end,
      - xorout is applied last.
    """
    mask = (1 << width) - 1
    crc = init
    for b in data:
        if refin:
            b = reflect(b, 8)
        for i in range(7, -1, -1):
            bit = (b >> i) & 1
            fb = ((crc >> (width - 1)) & 1) ^ bit
            crc = (crc << 1) & mask
            if fb:
                crc ^= poly
    return (reflect(crc, width) if refout else crc) ^ xorout

# ----------------------------------------------------------------------------
# 2. beat-serial model of the module semantics (no tables)
# ----------------------------------------------------------------------------

def beat_transform(word, dwidth, refin, lsb_first):
    """Apply the module's input conventions to one DATA_WIDTH-bit beat.

    - REFIN: reflect the bits inside every 8-bit byte lane (lanes counted
      from the LSB end; a partial top lane of (DATA_WIDTH % 8) bits, if any,
      is reflected within itself).
    - LSB_FIRST: reverse the order of the 8-bit byte lanes (little-endian
      beat: the LSB lane carries the first byte of the stream). A partial
      top lane keeps its place. No effect for DATA_WIDTH <= 8.
    """
    if refin:
        nb = dwidth // 8
        pw = dwidth % 8
        w2 = 0
        for b in range(nb):
            w2 |= reflect((word >> (8 * b)) & 0xFF, 8) << (8 * b)
        if pw:
            part = (word >> (8 * nb)) & ((1 << pw) - 1)
            w2 |= reflect(part, pw) << (8 * nb)
        word = w2
    if lsb_first:
        nb = dwidth // 8
        w2 = word
        for b in range(nb):
            src = (word >> (8 * (nb - 1 - b))) & 0xFF
            w2 = (w2 & ~(0xFF << (8 * b))) | (src << (8 * b))
        word = w2
    return word


def finalize_reg(reg, width, refout, xorout):
    """Final output mapping of the internal register (REFOUT then XOR_OUT)."""
    return (reflect(reg, width) if refout else reg) ^ xorout


def crc_beats(beats, dwidth, width, poly, init, refin, refout, xorout, lsb_first):
    """Bit-serial absorption of DATA_WIDTH-bit beats, MSB bit first."""
    mask = (1 << width) - 1
    crc = init
    for word in beats:
        word = beat_transform(word, dwidth, refin, lsb_first)
        for i in range(dwidth - 1, -1, -1):
            bit = (word >> i) & 1
            fb = ((crc >> (width - 1)) & 1) ^ bit
            crc = (crc << 1) & mask
            if fb:
                crc ^= poly
    return finalize_reg(crc, width, refout, xorout)

# ----------------------------------------------------------------------------
# 3. Sarwate table-folding model (mirrors the RTL exactly)
# ----------------------------------------------------------------------------

def make_table(width, poly, chunk_w):
    """Sarwate lookup table.

    table[i] = value the all-zero register takes after `chunk_w` LFSR steps
    with the index i placed in its top `chunk_w` bits, i.e. the CRC-register
    contribution of absorbing the chunk i.
    """
    mask = (1 << width) - 1
    tbl = []
    for idx in range(1 << chunk_w):
        t = (idx << (width - chunk_w)) & mask
        for _ in range(chunk_w):
            if (t >> (width - 1)) & 1:
                t = ((t << 1) ^ poly) & mask
            else:
                t = (t << 1) & mask
        tbl.append(t)
    return tbl


def crc_beats_table(beats, dwidth, width, poly, init, refin, refout, xorout, lsb_first):
    """Table-folding absorption, structurally identical to the RTL."""
    chunk_w = min(dwidth, width, 8)
    full = dwidth // chunk_w
    tail = dwidth % chunk_w
    mask = (1 << width) - 1
    tmain = make_table(width, poly, chunk_w)
    ttail = make_table(width, poly, tail) if tail else None

    crc = init
    for word in beats:
        word = beat_transform(word, dwidth, refin, lsb_first)
        # fold the full chunks, MSB-most chunk first
        for k in range(full):
            c = (word >> (dwidth - (k + 1) * chunk_w)) & ((1 << chunk_w) - 1)
            idx = ((crc >> (width - chunk_w)) ^ c) & ((1 << chunk_w) - 1)
            crc = ((crc << chunk_w) & mask) ^ tmain[idx]
        # fold the tail chunk, if any
        if tail:
            c = word & ((1 << tail) - 1)
            idx = ((crc >> (width - tail)) ^ c) & ((1 << tail) - 1)
            crc = ((crc << tail) & mask) ^ ttail[idx]
    return finalize_reg(crc, width, refout, xorout)

# ----------------------------------------------------------------------------
# beat packing helpers
# ----------------------------------------------------------------------------

def bytes_to_beats_be(data, dwidth):
    """Big-endian packing: byte stream -> MSB-first bit stream -> D-bit beats."""
    total = len(data) * 8
    if total % dwidth != 0:
        raise ValueError("message length not a multiple of the beat width")
    v = int.from_bytes(data, 'big')
    beats = []
    for i in range(total // dwidth):
        shift = total - (i + 1) * dwidth
        beats.append((v >> shift) & ((1 << dwidth) - 1))
    return beats


def bytes_to_beats_le(data, dwidth):
    """Little-endian byte-lane packing: first byte into the LSB lane."""
    nl = dwidth // 8
    if nl == 0 or len(data) % nl != 0:
        raise ValueError("message length not a multiple of the beat size")
    beats = []
    for i in range(0, len(data), nl):
        w = 0
        for j, b in enumerate(data[i:i + nl]):
            w |= b << (8 * j)
        beats.append(w)
    return beats

# ----------------------------------------------------------------------------
# self-tests
# ----------------------------------------------------------------------------

MSG9 = b"123456789"
MSG12 = b"123456789ABC"

CATALOG = [
    # name, width, poly, init, refin, refout, xorout, check value of "123456789"
    ("CRC-32",            32, 0x04C11DB7, 0xFFFFFFFF, True,  True,  0xFFFFFFFF, 0xCBF43926),
    ("CRC-16/MODBUS",     16, 0x8005,     0xFFFF,     True,  True,  0x0000,     0x4B37),
    ("CRC-16/CCITT-FALSE",16, 0x1021,     0xFFFF,     False, False, 0x0000,     0x29B1),
    ("CRC-16/XMODEM",     16, 0x1021,     0x0000,     False, False, 0x0000,     0x31C3),
    ("CRC-8/SMBUS",        8, 0x07,       0x00,       False, False, 0x00,       0xF4),
    ("CRC-8/ROHC",         8, 0x07,       0xFF,       True,  True,  0x00,       0xD0),
    ("CRC-32/MPEG-2",     32, 0x04C11DB7, 0xFFFFFFFF, False, False, 0x00000000, 0x0376E6E7),
    ("CRC-16/ARC",        16, 0x8005,     0x0000,     True,  True,  0x0000,     0xBB3D),
    ("CRC-6/GSM",          6, 0x2F,       0x00,       False, False, 0x3F,       0x13),
]


def test_catalog():
    """Layer 1: the byte-stream reference must reproduce published check values."""
    for name, w, poly, init, refin, refout, xorout, check in CATALOG:
        got = crc_bytes(MSG9, w, poly, init, refin, refout, xorout)
        assert got == check, "catalog mismatch %s: got 0x%X expected 0x%X" % (name, got, check)
    print("[OK] layer 1: byte-stream reference matches %d CRC catalog check values"
          % len(CATALOG))


def test_beat_vs_bytes():
    """Layer 2: the per-beat semantics must agree with the byte-stream model
    for every byte-aligned beat width and both packing conventions."""
    msg = MSG12  # 12 bytes: divisible by 8, 12, 16, 24, 32, 48, 96
    cases = []
    for dwidth in (8, 12, 16, 24, 32, 48):
        cases.append(("D%d BE" % dwidth, bytes_to_beats_be(msg, dwidth), False))
        if dwidth % 8 == 0:
            cases.append(("D%d LE" % dwidth, bytes_to_beats_le(msg, dwidth), True))
    n = 0
    for name, w, poly, init, refin, refout, xorout, _check in CATALOG:
        for case, beats, lsb in cases:
            dwidth = int(case[1:case.index(' ')])
            # per-beat-lane reflection coincides with per-stream-byte reflection
            # only when the beat width is a byte multiple; skip refin configs
            # otherwise (they are still covered by layer 3)
            if refin and (dwidth % 8) != 0:
                continue
            got = crc_beats(beats, dwidth, w, poly, init, refin, refout, xorout, lsb)
            exp = crc_bytes(msg, w, poly, init, refin, refout, xorout)
            assert got == exp, "beat/byte mismatch %s %s: 0x%X vs 0x%X" % (name, case, got, exp)
            n += 1
    print("[OK] layer 2: %d beat-width/packing cases agree with the byte-stream model" % n)


def test_fuzz_table_vs_bitserial():
    """Layer 3: Sarwate table-folding == bit-serial, over random configs."""
    rng = random.Random(0xC0FFEE)
    n = 0
    for _ in range(3000):
        w = rng.choice([1, 2, 3, 4, 5, 6, 6, 7, 8, 8, 12, 16, 16, 24, 32, 32,
                        rng.randint(1, 40)])
        d = rng.choice([1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 15, 16, 17, 20,
                        23, 24, 25, 31, 32, 33, 40, 48, 64])
        poly = rng.randrange(1, 1 << w)
        init = rng.randrange(0, 1 << w)
        xor = rng.randrange(0, 1 << w)
        refin = rng.random() < 0.5
        refout = rng.random() < 0.5
        lsb = rng.random() < 0.5
        beats = [rng.randrange(0, 1 << d) for _ in range(rng.randint(1, 4))]
        a = crc_beats(beats, d, w, poly, init, refin, refout, xor, lsb)
        b = crc_beats_table(beats, d, w, poly, init, refin, refout, xor, lsb)
        assert a == b, ("fuzz mismatch w=%d d=%d poly=0x%X init=0x%X xor=0x%X "
                        "refin=%d refout=%d lsb=%d beats=%s: 0x%X vs 0x%X"
                        % (w, d, poly, init, xor, refin, refout, lsb, beats, a, b))
        n += 1
    print("[OK] layer 3: Sarwate table-folding matches bit-serial on %d random configs" % n)

# ----------------------------------------------------------------------------
# testbench expected-vector generation
# ----------------------------------------------------------------------------

CFGS = {
    # the eight catalog anchors plus the module's default parameter set
    'CRC32':  dict(w=32, poly=0x04C11DB7, init=0xFFFFFFFF, xor=0xFFFFFFFF, refin=True,  refout=True),
    'MODBUS': dict(w=16, poly=0x8005,     init=0xFFFF,     xor=0x0000,     refin=True,  refout=True),
    'CCITTF': dict(w=16, poly=0x1021,     init=0xFFFF,     xor=0x0000,     refin=False, refout=False),
    'XMODEM': dict(w=16, poly=0x1021,     init=0x0000,     xor=0x0000,     refin=False, refout=False),
    'SMBUS':  dict(w=8,  poly=0x07,       init=0x00,       xor=0x00,       refin=False, refout=False),
    'ROHC':   dict(w=8,  poly=0x07,       init=0xFF,       xor=0x00,       refin=True,  refout=True),
    'MPEG2':  dict(w=32, poly=0x04C11DB7, init=0xFFFFFFFF, xor=0x00000000, refin=False, refout=False),
    'ARC':    dict(w=16, poly=0x8005,     init=0x0000,     xor=0x0000,     refin=True,  refout=True),
    'USER6':  dict(w=6,  poly=0x2F,       init=0x2A,       xor=0x15,       refin=True,  refout=True),
}

# wide-beat configurations: (sv_name, cfg_key, dwidth, packing)
#   packing 'be' -> LSB_FIRST = 0, 'le' -> LSB_FIRST = 1
#   'be_refl' = big-endian packing of the byte-reflected stream (D=1, REFIN=0)
WIDE = [
    ('CRC32_D32_BE',  'CRC32',  32, 'be'),
    ('CRC32_D32_LE',  'CRC32',  32, 'le'),
    ('MPEG2_D32_BE',  'MPEG2',  32, 'be'),
    ('MPEG2_D12_BE',  'MPEG2',  12, 'be'),
    ('MODBUS_D16_LE', 'MODBUS', 16, 'le'),
    ('XMODEM_D24_BE', 'XMODEM', 24, 'be'),
    ('USER6_D16_BE',  'USER6',  16, 'be'),
    ('USER6_D6_BE',   'USER6',   6, 'be'),
    ('USER6_D3_BE',   'USER6',   3, 'be'),
    ('CCITTF_D4_BE',  'CCITTF',  4, 'be'),
    ('SMBUS_D3_BE',   'SMBUS',   3, 'be'),
    ('CRC32_D1_REF0', 'CRC32',   1, 'be_refl'),
    ('CRC32_D1_REF1', 'CRC32',   1, 'be'),
]


def emit_svh(path):
    """Emit crc_expected.svh with beats / expected values for the testbench."""
    L = []
    L.append("// crc_expected.svh -- AUTO-GENERATED by crc_ref.py, do not edit.")
    L.append("// Expected results for prim_CRC_Calculator_tb, produced by the")
    L.append("// independent bit-serial reference model (see crc_ref.py).")
    L.append("")
    L.append("localparam int unsigned N_MSG9  = %d;" % len(MSG9))
    L.append("localparam int unsigned N_MSG12 = %d;" % len(MSG12))
    L.append("localparam logic [7:0] MSG9  [N_MSG9]  = '{%s};"
             % ", ".join("8'h%02X" % b for b in MSG9))
    L.append("localparam logic [7:0] MSG12 [N_MSG12] = '{%s};"
             % ", ".join("8'h%02X" % b for b in MSG12))
    L.append("")

    # D=8 shared-bus configs: phase 1 (msg9, catalog anchors) and phase 2 (msg12)
    p2_val = {}
    for key, c in CFGS.items():
        rst = finalize_reg(c['init'], c['w'], c['refout'], c['xor'])
        L.append("localparam logic [%d:0] RST_%s = %d'h%X;"
                 % (c['w'] - 1, key, c['w'], rst))
    L.append("")
    for key, c in CFGS.items():
        base = dict(width=c['w'], poly=c['poly'], init=c['init'],
                    refin=c['refin'], refout=c['refout'], xorout=c['xor'], lsb_first=False)
        p1 = crc_beats_table(list(MSG9), 8, **base)
        a4 = crc_beats_table(list(MSG9[:4]), 8, **base)
        p2 = crc_beats_table(list(MSG12), 8, **base)
        L.append("localparam logic [%d:0] EXP_P1_%s = %d'h%X; // %s, msg9"
                 % (c['w'] - 1, key, c['w'], p1, c['w']))
        L.append("localparam logic [%d:0] EXP_P1_%s_A4 = %d'h%X; // after 4 bytes"
                 % (c['w'] - 1, key, c['w'], a4))
        L.append("localparam logic [%d:0] EXP_P2_%s = %d'h%X; // msg12"
                 % (c['w'] - 1, key, c['w'], p2))
        p2_val[key] = p2
    L.append("")

    # wide-beat configs
    exp_cache = {}
    for name, key, dwidth, packing in WIDE:
        c = CFGS[key]
        lsb = (packing == 'le')
        if packing == 'be_refl':
            beats = bytes_to_beats_be(reflect_bytes(MSG12), dwidth)
            refin = False
        else:
            beats = (bytes_to_beats_le(MSG12, dwidth) if packing == 'le'
                     else bytes_to_beats_be(MSG12, dwidth))
            refin = c['refin']
        exp = crc_beats_table(beats, dwidth, c['w'], c['poly'], c['init'],
                              refin, c['refout'], c['xor'], lsb)
        rst = finalize_reg(c['init'], c['w'], c['refout'], c['xor'])
        exp_cache[name] = exp
        L.append("localparam int unsigned N_BEAT_%s = %d;" % (name, len(beats)))
        L.append("localparam logic [%d:0] BEAT_%s [N_BEAT_%s] = '{%s};"
                 % (dwidth - 1, name, name,
                    ", ".join("%d'h%X" % (dwidth, b) for b in beats)))
        L.append("localparam logic [%d:0] EXP_%s = %d'h%X;"
                 % (c['w'] - 1, name, c['w'], exp))
        L.append("localparam logic [%d:0] RST_%s = %d'h%X;"
                 % (c['w'] - 1, name, c['w'], rst))
        L.append("")

    # cross-consistency of emitted expectations:
    # byte-multiple beat widths and both packings must agree with the D=8 result
    def E(nm):
        return exp_cache[nm]
    assert E('CRC32_D32_BE') == E('CRC32_D32_LE') == E('CRC32_D1_REF0') == p2_val['CRC32'], \
        "CRC-32 expectations must be packing-independent"
    assert E('MPEG2_D32_BE') == E('MPEG2_D12_BE') == p2_val['MPEG2'], \
        "MPEG-2 expectations must be packing-independent (incl. D=12 tail case)"
    assert E('MODBUS_D16_LE') == p2_val['MODBUS'], "MODBUS D16 LE mismatch"
    assert E('XMODEM_D24_BE') == p2_val['XMODEM'], "XMODEM D24 mismatch"
    assert E('CCITTF_D4_BE') == p2_val['CCITTF'], "CCITT-FALSE D4 mismatch"
    assert E('SMBUS_D3_BE') == p2_val['SMBUS'], "SMBUS D3 mismatch"
    # USER6_D16_BE / USER6_D6_BE / USER6_D3_BE / CRC32_D1_REF1 carry model-only
    # expectations (non-byte-multiple widths / reflected raw stream); their
    # correctness is guaranteed by the layer-3 fuzz equivalence.
    print("[OK] emitted %s" % path)

    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")

# ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--emit', default=None, help='output .svh path')
    args = ap.parse_args()
    test_catalog()
    test_beat_vs_bytes()
    test_fuzz_table_vs_bitserial()
    if args.emit:
        emit_svh(args.emit)
    print("[DONE] all reference-model self-tests passed")


if __name__ == '__main__':
    main()
