# IMU 惯导数据处理项目

本项目用于处理惯性测量单元（IMU）的 ASC 数据，包含加速度计标定、陀螺仪标定和初始对准功能。程序使用 C++ 编写，通过配置文件指定处理模式、输入数据和结果保存路径。

## 功能

| 模式 | 功能 | 说明 |
| --- | --- | --- |
| `MODE=0` | 加速度计标定 | 使用六位置数据估计加速度计误差，并生成补偿结果 |
| `MODE=1` | 陀螺仪标定 | 使用各轴正反向转动数据估计陀螺仪误差，并生成补偿结果 |
| `MODE=2` | 初始对准 | 处理对准数据，输出全时段、逐秒结果及偏差等数据 |

运行模式及输入、输出文件路径在 `src/config.txt` 中配置。

## 项目结构

| 路径 | 内容 |
| --- | --- |
| `src/` | C++ 源代码、配置文件和可执行程序输出目录 |
| `Experiment1/` | 实验指导材料、数据转换参考资料及实验数据 |
| `work/asc/` | 标定和初始对准使用的 ASC 输入数据 |
| `work/results/` | 程序生成的标定与对准结果 |
| `work/figures/` | 实验报告图片、绘图脚本及相关数据 |
| `work/report/` | LaTeX 实验报告源码和编译输出目录 |

## 环境要求

- 支持 C++17 的 `g++`
- Eigen 3.x
- 如需重新生成报告图片：Python、NumPy、Matplotlib 和 Node.js
- 如需编译实验报告：XeLaTeX 和 `latexmk`

## 编译程序

在项目根目录打开终端，使用以下命令编译：

```sh
g++ -std=c++17 -Wall -Wextra -I <Eigen目录> \
  src/main.cpp src/ReadFile.cpp src/ReadConfig.cpp \
  src/ErrorCalibration.cpp src/CoarseAlignment.cpp \
  -o src/output/IMU.exe
```

将 `<Eigen目录>` 替换为本机 Eigen 的实际路径。Windows 下也可以使用 VS Code 中的 `Compile IMU` 构建任务；如果 Eigen 不在 `D:/ThirdParty/eigen-3.3.8`，请相应修改 `.vscode/tasks.json` 中的包含路径。

## 配置与运行

默认配置文件为 `src/config.txt`。程序从**项目根目录**运行时，可直接使用默认配置：

```sh
.\src\output\IMU.exe
```

也可以显式指定配置文件：

```sh
.\src\output\IMU.exe src\config.txt
```

选择需要的处理模式，设置对应的输入数据和输出路径：

```ini
MODE=0
```

`MODE` 可设为 `0`、`1` 或 `2`，分别对应加速度计标定、陀螺仪标定和初始对准。配置文件中的相对路径以项目根目录为基准；修改数据文件位置后，也要同步更新相应配置项。

## 实验数据准备

实验采集的二进制数据需要先使用 NovAtel Convert 转换为 ASC 文件，再放入 `work/asc/` 下对应的目录。`Experiment1/` 中提供 ASC 数据记录转换参考资料和实验指导文档。

标定实验使用 NovAtel-FSAS 数据，对准实验使用 NovAtel-100C 数据。两类设备的数据比例因子不同；转换 ASC 数据时，请按照转换参考表选择对应设备和参数。

## 实验报告与图片

报告主文件为 `work/report/main.tex`。在 `work/report/` 目录中运行以下命令编译 PDF：

```sh
latexmk -xelatex -outdir=build main.tex
```

如需重新生成全部报告统计数据和绘图，在项目根目录运行：

```sh
node work/figures/build_report_figures.mjs
```

也可以运行 `work/figures/` 中对应的 `plot_*.py` 脚本单独重画一张图。更多图片脚本和绘图说明见 `work/figures/README.md`。