"""alignment_observations.png 的独立绘图脚本。
直接运行: python work/figures/plot_alignment_observations.py
在 render() 中调整尺寸、颜色、标题、坐标和图例。数据计算共用 report_data.py。
"""
import numpy as np
from plot_common import plt, run_one, format_figure
from report_data import DT, G, OMEGA, LAT, blocks


def render(context):
    tb = context['tb']
    mean6 = context['mean6']
    fig,axs=plt.subplots(2,1,figsize=(10.5,6.8),sharex=True)
    for j in range(3):axs[0].plot(tb,mean6[:,j],label='XYZ'[j],lw=2)
    for j in range(3):axs[1].plot(tb,np.rad2deg(mean6[:,j+3])*3600,label='XYZ'[j],lw=2)
    axs[0].set_ylabel('每秒平均比力 / (m/s²)');axs[1].set_ylabel('每秒平均角速度 / (°/h)');axs[1].set_xlabel('相对时间 / s')
    for ax in axs:ax.legend(ncol=3);ax.grid(alpha=.2)
    format_figure(fig, time_series=True)
    return fig


if __name__ == '__main__':
    run_one('alignment_observations')
