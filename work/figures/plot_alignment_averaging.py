"""alignment_averaging.png 的独立绘图脚本。
直接运行: python work/figures/plot_alignment_averaging.py
在 render() 中调整尺寸、颜色、标题、坐标和图例。数据计算共用 report_data.py。
"""
import numpy as np
from plot_common import plt, run_one, format_figure
from report_data import DT, G, OMEGA, LAT, blocks


def render(context):
    convergence = context['convergence']
    fig,axs=plt.subplots(1,2,figsize=(11.2,4.2))
    cv=np.array([c['difference_deg'] for c in convergence]);times=[c['T_s'] for c in convergence]
    for j in range(3):axs[0].semilogx(times,cv[:,j],marker='o',ms=3,label=['航向','俯仰','横滚'][j])
    axs[0].set(xlabel='累计平均时长 / s',ylabel='相对全段姿态差 / °');axs[0].legend()
    T=np.geomspace(.005,2000,400)
    bias=.05/(np.rad2deg(OMEGA)*3600*np.cos(LAT))*180/np.pi
    noise=(.005/np.sqrt(T/3600))/(np.rad2deg(OMEGA)*3600*np.cos(LAT))*180/np.pi
    axs[1].loglog(T,noise,label='ARW=0.005°/√h');axs[1].axhline(bias,color='#bb5a35',ls='--',label='零偏=0.05°/h')
    axs[1].set(xlabel='平均时长 / s',ylabel='理论航向误差量级 / °');axs[1].legend(fontsize=9)
    for ax in axs:ax.grid(alpha=.2)
    format_figure(fig, time_series=False)
    return fig


if __name__ == '__main__':
    run_one('alignment_averaging')
