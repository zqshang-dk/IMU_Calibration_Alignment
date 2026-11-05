"""Shared style and direct single-figure entry point; no data-file I/O in Python."""
import os
import sys
sys.dont_write_bytecode = True
import subprocess
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Microsoft YaHei','SimHei','DejaVu Sans'],
                     'axes.unicode_minus':False,'font.size':14,'axes.spines.top':True,
                     'axes.spines.right':True,'xtick.top':False,'ytick.right':False,
                     'figure.dpi':600,'savefig.dpi':600})

# Time-series display only; calibration and statistics still use full valid records.
TIME_LIMITS = (0, 75)


def format_figure(fig, time_series=False):
    """Closed axes, bottom/left ticks, and one shared legend above all panels."""
    legends = {}
    for ax in fig.axes:
        for spine in ax.spines.values():
            spine.set_visible(True)
        ax.tick_params(which='both', top=False, right=False, labeltop=False, labelright=False)
        if time_series:
            ax.set_xlim(*TIME_LIMITS)
            ax.set_xticks(range(TIME_LIMITS[0], TIME_LIMITS[1] + 1, 15))
        handles, labels = ax.get_legend_handles_labels()
        for handle, label in zip(handles, labels):
            legends.setdefault(label, handle)
        if ax.get_legend() is not None:
            ax.get_legend().remove()
    if legends:
        fig.legend(list(legends.values()), list(legends), loc='upper center',
                   bbox_to_anchor=(0.5, 0.995), ncol=min(5, len(legends)),
                   frameon=False, fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.93 if legends else 1))


def run_one(name):
    bridge = str(Path(__file__).with_name('build_report_figures.mjs'))
    env = dict(os.environ, IMU_REPORT_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE='1')
    raise SystemExit(subprocess.call(['node', bridge, '--figure', name], env=env))
