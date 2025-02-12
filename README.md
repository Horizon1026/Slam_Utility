# Slam Utility
Defined some basic datatype or operations for slam algorithm.

# Components
- [x] Datatype for matrix, image, image pyramid and so on.
- [x] Simple log api.
- [x] Simple math lib.
- [x] Memory manager.
- [x] Basic operations.
- [ ] Data struct for slam.
    - [x] Visual covisible graph.
    - [x] Object pool.
    - [x] Circular queue.
    - [x] Kd tree.
- [x] Tick tock timer.
- [x] Scripts for all repos based on Slam_Utility.

# Dependence
- Eigen3 (>= 3.3.7)
- Visualizor2D (only for test)
- Visualizor3D (only for test)

# Compile and Run
- 第三方仓库的话需要自行 apt-get install 安装
- 拉取 Dependence 中的源码，在当前 repo 中创建 build 文件夹，执行标准 cmake 过程即可
```bash
mkdir build
cmake ..
make -j
```
- 编译成功的可执行文件就在 build 中，具体有哪些可执行文件可参考 run.sh 中的列举。可以直接运行 run.sh 来依次执行所有可执行文件

```bash
sh run.sh
```

# Tips
- 欢迎一起交流学习，不同意商用；
- 这是一个 basic 的基础“积木”仓库，会被较多更高级的“积木”仓库所依赖，更新频率一般不会很高；
