"""acc_compensation_y.png 的独立绘图脚本。
直接运行: python work/figures/plot_acc_compensation_y.py
在 render() 中调整尺寸、颜色、标题、坐标和图例。数据计算共用 report_data.py。
"""
import numpy as np
from plot_common import plt, run_one, format_figure
from report_data import DT, G, OMEGA, LAT, blocks


def render(context):
    data = context['data']
    names = context['names']
    b = context['b']
    inv = context['inv']
    truth = context['truth']
    axis = 1
    fig,axs=plt.subplots(2,3,figsize=(12,6))
    for sign in range(2):
        k=axis*2+sign;t,f,w,_=data[names[k]];c=(f-b)@inv.T
        tb,fb,_=blocks(t,np.column_stack([f-truth[k],c-truth[k]]))
        for j in range(3):
            ax=axs[sign,j]
            ax.plot(tb,fb[:,j]*1000,label='补偿前',lw=2,color='#bb5a35')
            ax.plot(tb,fb[:,j+3]*1000,label='补偿后',lw=2,color='#126b91')
            ax.axhline(0,color='black',ls=':',lw=.7);ax.axvspan(0,5,color='grey',alpha=.12)
            ax.axvspan(t[-1]-t[0]-5,t[-1]-t[0],color='grey',alpha=.12)
            ax.set(title=f'{"XYZ"[axis]}轴{"朝上" if sign==0 else "朝下"}：{"XYZ"[j]}分量',xlabel='相对时间 / s',ylabel='比力残差 / (mm/s²)')
            ax.grid(alpha=.2)
    axs[0,0].legend(ncol=2,fontsize=9)
    format_figure(fig, time_series=True)
    return fig


if __name__ == '__main__':
    run_one('acc_compensation_y')
