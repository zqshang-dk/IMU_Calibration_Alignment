"""gyro_rotation_windows.png 的独立绘图脚本。
直接运行: python work/figures/plot_gyro_rotation_windows.py
在 render() 中调整尺寸、颜色、标题、坐标和图例。数据计算共用 report_data.py。
"""
import numpy as np
from plot_common import plt, run_one, format_figure
from report_data import DT, G, OMEGA, LAT, blocks


def render(context):
    data = context['data']
    windows = context['windows']
    fig,axs=plt.subplots(3,2,figsize=(11.8,8.2))
    for axis,j,name,start,stop in windows:
        t,_,w,_=data[name]
        rate=np.rad2deg(w[:,axis])
        ax=axs[axis,j];ax.plot(t-t[0],rate,color='#126b91',lw=2)
        ax.axvspan(t[start]-t[0],t[stop-1]-t[0],color='#e6ac39',alpha=.25,label='积分区间')
        ax.set(title=f'{"XYZ"[axis]}轴 {"正转" if j==0 else "反转"}',xlabel='相对时间 / s',ylabel='主轴角速度 / (°/s)');ax.grid(alpha=.2)
    axs[0,0].legend()
    format_figure(fig, time_series=True)
    return fig


if __name__ == '__main__':
    run_one('gyro_rotation_windows')
