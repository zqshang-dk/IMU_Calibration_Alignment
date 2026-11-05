"""六位置平均比力残差：每个分量各有补偿前、补偿后两条曲线。
运行：python work/figures/plot_acc_mean_residuals.py
均值沿用完整有效静态区间（首尾各去 5 s），不按绘图的 0–75 s 截断。
"""
import numpy as np
from plot_common import plt, run_one, format_figure


def render(context):
    stats = context['result']['accelerometer']
    reference = np.asarray(stats['reference'])
    before = (np.asarray(stats['mean_raw']) - reference) * 1000
    after = (np.asarray(stats['mean_compensated']) - reference) * 1000
    positions = np.arange(6)
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.2))
    for j, ax in enumerate(axes):
        ax.plot(positions, before[:, j], 'o-', color='#bb5a35', lw=1.3,
                ms=4, label='补偿前')
        ax.plot(positions, after[:, j], 's-', color='#126b91', lw=1.3,
                ms=4, label='补偿后')
        ax.axhline(0, color='black', ls=':', lw=0.7)
        ax.set_xticks(positions, ['X+', 'X−', 'Y+', 'Y−', 'Z+', 'Z−'])
        ax.set(title=f'{"XYZ"[j]}分量', xlabel='静态位置',
               ylabel='平均比力残差 / (mm/s²)')
        ax.grid(alpha=0.2)
    format_figure(fig)
    return fig


if __name__ == '__main__':
    run_one('acc_mean_residuals')
