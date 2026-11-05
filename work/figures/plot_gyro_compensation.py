"""gyro_compensation.png 的独立绘图脚本。
直接运行: python work/figures/plot_gyro_compensation.py
在 render() 中调整尺寸、颜色、标题、坐标和图例。数据计算共用 report_data.py。
"""
import numpy as np
from plot_common import plt, run_one, format_figure
from report_data import DT, G, OMEGA, LAT, blocks


def render(context):
    data = context['data']
    rot = context['rot']
    windows = context['windows']
    bg = context['bg']
    sg = context['sg']
    fig,axs=plt.subplots(3,2,figsize=(11.8,8.2))
    for row,(axis,j,name,start,stop) in zip(rot,windows):
        t,_,w,_=data[name];rate=np.rad2deg(w[:,axis]);earth=-np.rad2deg(OMEGA*np.sin(LAT))
        comp_rate=(rate-np.rad2deg(bg[axis]))/(1+sg[axis])-earth
        ax=axs[axis,j];tt=t-t[0]
        # The compensated-minus-raw difference is visible even when the traces overlap.
        ax.plot(tt,rate,label='补偿前',color='#bb5a35',lw=2)
        ax.plot(tt,comp_rate,label='补偿后（去地球项）',color='#126b91',lw=2)
        ax.set(title=f'{"XYZ"[axis]}轴 {"正转" if j==0 else "反转"}',xlabel='相对时间 / s',ylabel='角速度 / (°/s)');ax.grid(alpha=.2)
    axs[0,0].legend(fontsize=14)
    format_figure(fig, time_series=True)
    return fig


if __name__ == '__main__':
    run_one('gyro_compensation')
