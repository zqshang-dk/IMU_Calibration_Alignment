# 报告图片调整说明

每张实验图的绘图代码单独放在下列 Python 脚本中。在对应文件的 `render(context)` 内调整尺寸、颜色、线宽、坐标范围和图例，然后运行该脚本。

| 图片 | 独立脚本 |
| --- | --- |
| acc_compensation_x.png | plot_acc_compensation_x.py |
| acc_compensation_y.png | plot_acc_compensation_y.py |
| acc_compensation_z.png | plot_acc_compensation_z.py |
| acc_mean_residuals.png | plot_acc_mean_residuals.py |
| gyro_rotation_windows.png | plot_gyro_rotation_windows.py |
| gyro_compensation.png | plot_gyro_compensation.py |
| alignment_observations.png | plot_alignment_observations.py |
| alignment_second.png | plot_alignment_second.py |
| alignment_epoch.png | plot_alignment_epoch.py |
| alignment_averaging.png | plot_alignment_averaging.py |

在项目根目录单独重画，例如：

```text
python work/figures/plot_alignment_second.py
```

也可以从任意目录运行脚本的绝对路径。单图运行只更新该 PNG，统计 JSON 和其他图片不变。脚本依赖 Python（NumPy、Matplotlib）和 Node.js。

重算全部统计并重画全部 10 张图：

```text
node work/figures/build_report_figures.mjs
```

公共模块：`report_data.py` 为原始记录解析及数值计算，`plot_common.py` 为统一字体、四边框、顶部图例和单图启动入口，`report_figures.py` 为调度。实际绘图布局位于每个 `plot_*.py` 中。Node 桥负责只读原始 ASC 并将图片输出到当前目录。

时间序列图通过 `plot_common.py` 中的 `TIME_LIMITS = (0, 75)` 统一显示前 75 秒；上、右边框可见，但无刻度线。累计平均时长图保留完整横轴，以观察长时平均效果。显示区间不改变标定、积分和统计所用数据。新增六位置残差图对完整有效静态区间（首尾各去 5 秒）计算均值，每个分量各有补偿前、补偿后两条线。

以下 4 张图直接裁自 `IMU_Alignment_Calibration-main/实验指导书/惯性导航实验指导书.pdf`，不是 Python 绘制的示意图：

| 图片 | 指导书图号 | PDF物理页 |
| --- | --- | --- |
| guide_turntable.png | 图1 高精度三轴转台 | 5 |
| guide_six_positions.png | 图2 六位置与六位置旋转 | 8 |
| guide_gyro_rotation.png | 图3 Z轴朝上正反转 | 9 |
| guide_body_axes.png | 图4 载体坐标轴与姿态角 | 13 |

报告的图号自动连续编号，图注另外注明指导书原图号。原图内容和标注予以保留；正文说明了指导书示例与本批数据的区别。

修改图片后，在 `work/report` 内运行：

```text
latexmk -xelatex -outdir=build main.tex
```
