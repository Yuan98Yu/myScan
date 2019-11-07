#

[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

## 背景

本项目的目标是在3D重建项目中，为无人机采集图像提供路径规划。使用本项目，可以在虚拟环境中，模拟无人机在室外环境中的路径规划、自主导航和图像采集。

本项目目前支持以下4种路径规划：

1. 离线的路径规划：
    - 无人机在固定高度的安全平面，环绕场景一周，同时相机朝向场景中心拍照；
    - 或 无人机绕建筑物螺旋上升，同时同时相机朝向场景中心拍照；
2. 通过多轮迭代的方式划分场景：每次以3个点确定1个三角形，选择其中心点将其划分为3个小三角形。按迭代得到的点的顺序遍历场景，每个点的拍摄角度由包围其的三角形的三个点处的高度决定；
3. 将场景划分为多个同心圆，从外向内依次遍历同心圆。每个点的拍摄角度由距其最近的两点以及场景中心点的高度决定；
4. 将场景划分为多个同心圆，按圆的半径二分的策略依次遍历同心圆。每个点的拍摄角度由距其最近的三点的高度决定。

## 安装

1. 本项目使用 python3 编写，并依赖于 matplotlib, airsim。请确保 python 版本，并通过 pip 安装 matplotlib, airsim

    `pip install matplotlib airsim`

2. 本项目在由 Unreal 3 引擎构建的虚拟环境中运行，并且该虚拟环境需配置 Airsim 的插件以便支持无人机和其操作。编译虚拟环境这一步是可选的，具体的关于虚拟环境的编译步骤可参考 [Airsim 官方文档](https://microsoft.github.io/AirSim/)。

    虚拟环境可以直接使用现成的。Airsim 官方仓库在 releases 提供了多个可运行的环境。下载地址：[](https://github.com/microsoft/AirSim/releases)

3. 本项目的输出仅为图片集，为了生成 3D 模型，需将图片集交由 3D 重建软件处理。3D 重建的步骤可分为 SFM、MVS、Surface Reconstruction，我们推荐使用的3D重建软件为 COLMAP，它集成了这3个步骤。最终生成的 3D 模型为 .ply 格式，可使用 meshlab 查看。

## 使用说明

### 运行步骤

1. 运行**虚拟环境**

2. 在项目根目录运行 `src/main.py`：需指定**配置文件**，若不指定，则设为默认值 `configs/config.json`

    `python src/main.py configs/dete-circle-binary.json`

3. 若安装了 COLMAP，可使用 COLMAP 处理采集到的图像集，重建 3D 模型。COLMAP 既提供 CLI，也提供界面，可根据需要使用。

### 配置文件

配置针对的是 一次无人机的采集图像任务，具体如下：

1. `type`   ：路径规划类型，目前支持`"offline"`， `"dete"` ,  `"dete_circle"`, `"dete_circle_binary"`
2. `safety_surface`：待重建场景的安全平面
3. `point_num`：场景的平面图网格化后的网格点数量，对于`safty_surface`，其等价于最终得到的照片数量
4. `image_dir`：照片的输出路径

对于`"dete"`,  `dete_circle`，`dete_circle_binary`，需额外指定：

- `fov`：与相机拍摄角度相关

对于`"dete"`需额外指定：

- `round_num`：迭代轮数

对于`"dete"`,  `"dete_circle"`，需额外指定：

- `circle_num`：同心圆数量

