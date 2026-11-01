"""制御周期の記録（モニターの Log タブで保存した CSV）を解析する。

使い方:
    python tools/analyze_log.py <CSVファイル> [--axis pitch|roll]

出力（CSV と同じフォルダ）:
    <名前>.png  目標角と姿勢角・角速度・PID の各項・サーボ出力の時系列
    <名前>.txt  下に表示する数値のまとめ

--axis を省くと、CSV の「# rig:」（台の試験）の軸を解析する。台の試験でなければピッチとロールの両方。
"""

import argparse
import math
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# タイトルのメモは日本語のことがあるので、Windows の日本語フォントを優先する
plt.rcParams["font.family"] = ["Yu Gothic", "MS Gothic", "DejaVu Sans"]

AXES = ("pitch", "roll", "yaw")
SERVO_LIMIT_MARGIN = 0.5  # [%] 0 や 100 からこれ以内なら張り付いているとみなす


def read_log(path):
    meta = {}
    n_comment = 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.startswith("#"):
                break
            n_comment += 1
            body = line[1:].strip()
            if ":" in body:
                key, value = body.split(":", 1)
                meta[key.strip()] = value.strip()
    # 先頭のコメント行を飛ばし、その次の行を列名として読む
    data = np.genfromtxt(path, delimiter=",", names=True, skip_header=n_comment, encoding="utf-8")
    return meta, data


def response_metrics(t, err):
    """誤差 err（目標 − 角度）[rad] の応答の指標。最大のずれの時刻から後を見る"""
    m = {}
    i_peak = int(np.argmax(np.abs(err)))
    peak = err[i_peak]
    m["peak_deg"] = math.degrees(peak)
    m["t_peak"] = t[i_peak]

    after = err[i_peak:]
    t_after = t[i_peak:]
    # 行き過ぎ: ずれと逆向きに最大どこまで行ったか
    opposite = -np.sign(peak) * after
    m["overshoot_deg"] = math.degrees(max(0.0, float(np.max(opposite))))
    m["overshoot_pct"] = 100.0 * m["overshoot_deg"] / abs(m["peak_deg"]) if peak != 0 else 0.0

    # 落ち着くまでの時間: |誤差| が帯（1° か ピークの10% の大きい方）に入ったまま出なくなるまで
    band = max(math.radians(1.0), 0.1 * abs(peak))
    outside = np.nonzero(np.abs(after) > band)[0]
    if len(outside) == 0:
        m["settle_s"] = 0.0
    elif outside[-1] + 1 < len(after):
        m["settle_s"] = t_after[outside[-1] + 1] - t_after[0]
    else:
        m["settle_s"] = float("nan")  # 記録の終わりまで落ち着かなかった
    m["settle_band_deg"] = math.degrees(band)

    # 振動: 落ち着いた値のまわりの山と谷を拾い、間隔から周波数、隣り合う山の比から減衰比を出す。
    # 小さな揺れ（ノイズ）を数えないよう、帯（0.5°）を越えて反対側へ行ったときだけ次の山・谷とする
    centered = after - np.mean(after[len(after) * 3 // 4:])
    hyst = math.radians(0.5)
    extrema = []  # (index, 値)
    side = 0
    for i, v in enumerate(centered):
        s = 1 if v > hyst else (-1 if v < -hyst else 0)
        if s == 0:
            continue
        if s != side:
            extrema.append((i, v))
            side = s
        elif abs(v) > abs(extrema[-1][1]):
            extrema[-1] = (i, v)
    if len(extrema) >= 3:
        times = [t_after[i] for i, _ in extrema]
        m["osc_hz"] = 1.0 / (2.0 * float(np.median(np.diff(times))))
        amps = [abs(v) for _, v in extrema]
        ratios = [amps[i + 1] / amps[i] for i in range(len(amps) - 1)]
        # 最初の半周期の比 r から、1周期の対数減衰率 δ = −2 ln r（手で押した直後の勢いが入らないよう、最初の山の後から）
        r = ratios[1] if len(ratios) >= 2 else ratios[0]
        if 0 < r < 1:
            delta = -2.0 * math.log(r)
            m["damping_ratio"] = delta / math.sqrt(4 * math.pi ** 2 + delta ** 2)
        m["extrema_deg"] = [round(math.degrees(v), 2) for _, v in extrema[:6]]
    return m


def dominant_freq(t, x, fmin=0.5):
    """x の主な周波数 [Hz]（fmin 以上で振幅が最大のもの）"""
    if len(x) < 16:
        return float("nan")
    dt = float(np.median(np.diff(t)))
    x = x - np.mean(x)
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    freqs = np.fft.rfftfreq(len(x), dt)
    mask = freqs >= fmin
    if not np.any(mask):
        return float("nan")
    return float(freqs[mask][np.argmax(spec[mask])])


def analyze(meta, d, axes_active):
    t = d["t_s"]
    lines = []
    add = lines.append
    add(f"記録: {meta.get('samples', len(t))} サンプル、周期 {meta.get('period_us', '?')} us、"
        f"制御モード {meta.get('control_mode', '?')}、台の試験 {meta.get('rig', 'none')}")
    if meta.get("memo"):
        add(f"メモ: {meta['memo']}")
    for ax in AXES:
        if f"pid_{ax}" in meta:
            add(f"PID {ax}: {meta[f'pid_{ax}']}")
    thr = d["throttle_pct"]
    add(f"スロットル: 平均 {np.mean(thr):.1f}%（{np.min(thr):.1f}〜{np.max(thr):.1f}%）")
    add("")

    for ax in axes_active:
        ang = d[f"{ax}_rad"]
        tgt = d[f"target_{ax}_rad"]
        rate = d[f"rate_{ax}_rads"]
        err = tgt - ang
        add(f"== {ax}（制御している軸）==")
        add(f"  誤差: 平均 {math.degrees(np.mean(err)):+.2f}°、RMS {math.degrees(np.sqrt(np.mean(err ** 2))):.2f}°、"
            f"最大 {math.degrees(np.max(np.abs(err))):.2f}°")
        m = response_metrics(t, err)
        add(f"  最大のずれ {m['peak_deg']:+.2f}°（t={m['t_peak']:.2f}s）→ 行き過ぎ {m['overshoot_deg']:.2f}°"
            f"（{m['overshoot_pct']:.0f}%）、落ち着くまで "
            + ("記録内に落ち着かず" if math.isnan(m["settle_s"]) else f"{m['settle_s']:.2f}s")
            + f"（帯 ±{m['settle_band_deg']:.1f}°）")
        if "osc_hz" in m:
            add(f"  振動: {m['osc_hz']:.2f} Hz" + (f"、減衰比 ζ≈{m['damping_ratio']:.2f}" if "damping_ratio" in m else "")
                + f"、山と谷（落ち着いた値から）{m['extrema_deg']}°")
        add(f"  角度のスペクトルの主な周波数: {dominant_freq(t, ang):.2f} Hz")
        n_last = max(1, len(t) // 4)
        add(f"  最後の1/4の誤差: 平均 {math.degrees(np.mean(err[-n_last:])):+.2f}°、ばらつき {math.degrees(np.std(err[-n_last:])):.2f}°")
        add(f"  角速度: 最大 {math.degrees(np.max(np.abs(rate))):.1f}°/s、高周波のばらつき（差分の標準偏差）"
            f"{math.degrees(np.std(np.diff(rate))):.2f}°/s")
        for term in ("P", "I", "D"):
            v = d[f"{ax}_{term}_pct"]
            add(f"  {term}項: 平均 {np.mean(v):+.2f}%、標準偏差 {np.std(v):.2f}%、最大 {np.max(np.abs(v)):.2f}%"
                + (f"、最後 {v[-1]:+.2f}%" if term == "I" else ""))
        add("")

    add("== 制御していない軸 ==")
    for ax in AXES:
        if ax in axes_active:
            continue
        ang = d[f"{ax}_rad"]
        rate = d[f"rate_{ax}_rads"]
        add(f"  {ax}: 角度 平均 {math.degrees(np.mean(ang)):+.2f}°、ばらつき {math.degrees(np.std(ang)):.2f}°、"
            f"角速度 最大 {math.degrees(np.max(np.abs(rate))):.1f}°/s")
    add("")

    add("== サーボ ==")
    for i in range(1, 5):
        s = d[f"servo{i}_pct"]
        sat = np.mean((s <= SERVO_LIMIT_MARGIN) | (s >= 100.0 - SERVO_LIMIT_MARGIN)) * 100.0
        add(f"  S{i}: 平均 {np.mean(s):.1f}%（{np.min(s):.1f}〜{np.max(s):.1f}%）、端に張り付き {sat:.1f}%")
    return "\n".join(lines)


def plot(d, axes_active, path, title):
    t = d["t_s"]
    fig, axs = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    colors = {"pitch": "tab:blue", "roll": "tab:orange", "yaw": "tab:green"}

    ax = axs[0]
    for name in ("pitch", "roll"):
        style = "-" if name in axes_active else ":"
        ax.plot(t, np.degrees(d[f"{name}_rad"]), style, color=colors[name], label=f"{name}")
        if name in axes_active:
            ax.plot(t, np.degrees(d[f"target_{name}_rad"]), "--", color=colors[name], alpha=0.6, label=f"{name} target")
    ax.set_ylabel("angle [deg]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axs[1]
    for name in ("pitch", "roll", "yaw"):
        style = "-" if name in axes_active else ":"
        ax.plot(t, np.degrees(d[f"rate_{name}_rads"]), style, color=colors[name], label=name, linewidth=0.8)
    ax.set_ylabel("rate [deg/s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axs[2]
    term_styles = {"P": "-", "I": "--", "D": ":"}
    for name in axes_active:
        for term, style in term_styles.items():
            ax.plot(t, d[f"{name}_{term}_pct"], style, color=colors[name], label=f"{name} {term}", linewidth=0.9)
    ax.set_ylabel("PID terms [%]")
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axs[3]
    for i in range(1, 5):
        ax.plot(t, d[f"servo{i}_pct"], label=f"S{i}", linewidth=0.9)
    ax.plot(t, d["throttle_pct"], "k--", label="throttle", linewidth=0.9)
    ax.set_ylabel("output [%]")
    ax.set_xlabel("time [s]")
    ax.set_ylim(-5, 105)
    ax.legend(loc="upper right", fontsize=8, ncol=5)
    ax.grid(True, alpha=0.3)

    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv")
    parser.add_argument("--axis", choices=("pitch", "roll"))
    args = parser.parse_args()

    meta, d = read_log(args.csv)
    if args.axis:
        axes_active = (args.axis,)
    elif meta.get("rig") in ("pitch", "roll"):
        axes_active = (meta["rig"],)
    else:
        axes_active = ("pitch", "roll")

    report = analyze(meta, d, axes_active)
    base = os.path.splitext(args.csv)[0]
    with open(base + ".txt", "w", encoding="utf-8") as f:
        f.write(report + "\n")
    plot(d, axes_active, base + ".png", os.path.basename(args.csv) + ("  " + meta["memo"] if meta.get("memo") else ""))
    print(report)
    print(f"\n図: {base}.png")


if __name__ == "__main__":
    sys.exit(main())
