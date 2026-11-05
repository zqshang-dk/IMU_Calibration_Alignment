"""Calculate once and dispatch to independent per-figure rendering scripts.
Run the Node bridge, or run any plot_*.py directly for just that figure.
"""
import sys
sys.dont_write_bytecode = True
import json
import io
import base64
import importlib
from report_data import calculate
from plot_common import plt

FIGURES = ["acc_compensation_x","acc_compensation_y","acc_compensation_z","acc_mean_residuals","gyro_rotation_windows","gyro_compensation","alignment_observations","alignment_second","alignment_epoch","alignment_averaging"]


def main():
    requested = sys.argv[1:]
    if requested and (len(requested) != 2 or requested[0] != '--figure' or requested[1] not in FIGURES):
        raise SystemExit('Usage: report_figures.py [--figure NAME]')
    selected = [requested[1]] if requested else FIGURES
    context = calculate(json.load(sys.stdin))
    result = context['result']
    for name in selected:
        module = importlib.import_module('plot_' + name)
        fig = module.render(context)
        buffer = io.BytesIO()
        fig.savefig(buffer, format='png', facecolor='white')
        result['images'][name + '.png'] = base64.b64encode(buffer.getvalue()).decode('ascii')
        plt.close(fig)
    json.dump(result, sys.stdout, ensure_ascii=False, allow_nan=False)


if __name__ == '__main__':
    main()
