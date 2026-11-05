#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
重绘报告的两张关键图片：
1) alignment_epoch_his.png   逐历元姿态：未连线散点 + 时角二维频数
2) alignment_averaging_his.png 累计平均结果 + 两种官方资料 ARW 假设

无需重新解析原始 ASC，不依赖 Node.js；直接读取项目中已有的计算成果。
从任何目录执行：
    python work/figures/redraw_improved_figures.py
依赖：numpy、matplotlib
重要：此脚本只改变绘图方式，不改变原有数值结果/统计口径。
"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from matplotlib import font_manager as fm

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
EPOCH = ROOT / 'work/results/alignment/Align_Epoch.txt'
STATS = HERE / 'report_statistics.json'

available_fonts = {f.name for f in fm.fontManager.ttflist}
CHINESE_FONT = next((font for font in ('Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC',
                                       'Noto Sans CJK JP') if font in available_fonts), 'DejaVu Sans')
plt.rcParams.update({
    'font.family': CHINESE_FONT,
    'font.sans-serif': [CHINESE_FONT, 'DejaVu Sans'],
    'axes.unicode_minus': False,
    'font.size': 12,
    'axes.spines.top': True, 'axes.spines.right': True,
    'savefig.dpi': 600,
})


def wrapped_difference(degrees):
    """统一 [-180°,180°) 归化，与源码角差计算保持一致。"""
    return (np.asarray(degrees) + 180.0) % 360.0 - 180.0


def epoch_plot(stat):
    raw = np.loadtxt(EPOCH, delimiter=',', comments='%', dtype=float)
    time = raw[:, 0] - raw[0, 0]
    selection = (time >= 0) & (time < 75)
    t = time[selection]
    angles = raw[selection, 1:4]
    whole = np.asarray(stat['alignment']['whole_yaw_pitch_roll_deg'], dtype=float)
    diffs = wrapped_difference(angles - whole)
    fig, axs = plt.subplots(
        3, 2,
        figsize=(12, 7),
        sharex='col',
        gridspec_kw={
            'width_ratios': [1, 1.3]
        }
    )
    names = ('航向', '俯仰', '横滚')
    colors = ('#126b91', '#bb5a35', '#58824a')
    heat = None
    for j in range(3):
        left, right = axs[j]
        # 不连接连续点：每个5ms姿态是独立双矢量定姿结果，而非平滑运动轨迹。
        left.scatter(t[::2], angles[::2, j], s=1.5, alpha=.16,
                     color=colors[j], linewidths=0, rasterized=True)
        left.axhline(whole[j], ls='--', color='#222222', lw=1.0)
        left.set_ylabel(f'{names[j]} / °')
        left.grid(alpha=.15)
        yl = (-180, 180) if j == 0 else ((-.45, .45) if j == 1 else (-.45, .45))
        left.set_ylim(*yl)
        # 颜色表达每个0.5s x 角度小格中的观测数，对高度离散的航向尤其有效。
        yrange = (-180, 180) if j == 0 else ((-.45, .45) if j == 1 else (-.45, .45))
        heat = right.hist2d(t, diffs[:, j], bins=(150, 90),
                            range=((0, 75), yrange),
                            norm=LogNorm(vmin=1), cmap='Blues', cmin=1)[3]
        right.axhline(0, color='#b04a28', lw=.9, ls='--')
        right.set_ylabel(f'{names[j]}差 / °')
    axs[0, 0].set_title('逐历元独立姿态解')
    axs[0, 1].set_title('相对整段解的差值频数')
    for ax in axs[-1]:
        ax.set_xlabel('相对时间 / s')
    for ax in axs.ravel():
        ax.set_xlim(0, 75)
        ax.set_xticks(np.arange(0, 76, 15))
        ax.tick_params(which='both', top=False, right=False)
    fig.subplots_adjust(left=.09, right=.94, bottom=.08, top=.93,
                        hspace=.13, wspace=.2)
    cb = fig.colorbar(heat, ax=list(axs[:, 1]), location='right', shrink=.90, pad=.02)
    cb.set_label('每个时角分箱的记录数')
    out = HERE / 'alignment_epoch_his.png'
    fig.savefig(out, facecolor='white')
    plt.close(fig)
    return out


def averaging_plot(stat):
    conv = stat['alignment']['convergence']
    t = np.array([s['T_s'] for s in conv], dtype=float)
    differences = np.array([s['difference_deg'] for s in conv], dtype=float)
    lat = np.deg2rad(30.531651244)
    omega = 7.292115e-5
    earth_horizontal_deg_h = np.rad2deg(omega) * 3600.0 * np.cos(lat)
    x = np.geomspace(.005, 2000, 350)
    fig, axs = plt.subplots(2, 1, figsize=(11.4, 7))
    labels = ('航向', '俯仰', '横滚')
    for j in range(3):
        axs[0].semilogx(t, differences[:, j], marker='o', markersize=3.3,
                       linewidth=1.2, label=labels[j])
    axs[0].axhline(0, color='black', linewidth=.7, linestyle='--')
    axs[0].set_xlabel('累计平均时长 / s')
    axs[0].set_ylabel('相对全段姿态差 / °')
    axs[0].legend(frameon=False)
    bias = (.05 / earth_horizontal_deg_h) * 180.0 / np.pi
    for arw, color, line in ((.005, '#126b91', '-'), (.012, '#58824a', '-.')):
        noise_deg = (arw / np.sqrt(x / 3600.0) / earth_horizontal_deg_h) * 180.0 / np.pi
        axs[1].loglog(x, noise_deg, color=color, ls=line, lw=1.55,
                      label=f'ARW={arw:g}°/√h')
    axs[1].axhline(bias, color='#bb5a35', ls='--', lw=1.2,
                   label='名义零偏=0.05°/h')
    axs[1].set_xlabel('平均时长 / s')
    axs[1].set_ylabel('理论航向误差量级 / °')
    axs[1].legend(loc='upper right', fontsize=12, frameon=False)
    for ax in axs:
        ax.grid(alpha=.18, which='both')
        ax.tick_params(which='both', top=False, right=False)
    fig.tight_layout(pad=1.4, w_pad=2)
    out = HERE / 'alignment_averaging_his.png'
    fig.savefig(out, facecolor='white')
    plt.close(fig)
    return out


if __name__ == '__main__':
    stats = json.loads(STATS.read_text(encoding='utf-8'))
    for out in (epoch_plot(stats), averaging_plot(stats)):
        print(f'已生成: {out}')
