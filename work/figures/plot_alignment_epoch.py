"""alignment_epoch.png 的独立绘图脚本。
直接运行: python work/figures/plot_alignment_epoch.py
在 render() 中调整尺寸、颜色、标题、坐标和图例。数据计算共用 report_data.py。
"""
import numpy as np
from plot_common import plt, run_one, format_figure
from report_data import DT, G, OMEGA, LAT, blocks


def render(context):
    t = context['t']
    tb = context['tb']
    whole = context['whole']
    epoch = context['epoch']
    wrap = context['wrap']
    tt = t-t[0]
    ang = epoch
    fig,axs=plt.subplots(3,2,figsize=(10.5,6),sharex=True)
    # All samples plotted; raster output avoids exporting hundreds of thousands of vector elements.
    for j in range(3):
        ax=axs[j,1]
        ax.plot(tt,wrap(ang[:,j]-whole[j]),lw=.35,color=['#126b91','#bb5a35','#58824a'][j])
        ax.axhline(0,color='black',lw=.7,ls=':');ax.set_ylabel(['航向差 / °','俯仰差 / °','横滚差 / °'][j]);ax.grid(alpha=.2)
        ax=axs[j,0]
        ax.plot(tt,ang[:,j],lw=.35,color=['#126b91','#bb5a35','#58824a'][j])
        ax.axhline(whole[j],color='black',lw=.7,ls=':');ax.set_ylabel(['航向 / °','俯仰 / °','横滚 / °'][j]);ax.grid(alpha=.2)
    axs[0,0].set_title('姿态角');axs[0,1].set_title('相对整段解的角度差')
    for ax in axs[-1]:ax.set_xlabel('相对时间 / s')
    format_figure(fig, time_series=True)
    return fig


if __name__ == '__main__':
    run_one('alignment_epoch')
