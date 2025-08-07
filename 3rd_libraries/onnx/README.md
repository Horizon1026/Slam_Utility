首先显卡驱动得有，在 Terminal 里面输入 `nvidia-smi` 确认显卡驱动是否正常。如果有如下输出则是正常的。
![nvidia-smi](illustration/nvidia-smi.png)

```bash
git clone --branch v1.8.0 --recursive https://github.com/Microsoft/onnxruntime
cd onnxruntime/
# 目前有两个互不兼容的版本，1.15.0和1.8.0，选更新的先
git checkout v1.8.0
# 不使用cuda
./build.sh --skip_tests --config Release --build_shared_lib --parallel
# 使用cuda
./build.sh --skip_tests --use_cuda --config Release --build_shared_lib --parallel --cuda_home /usr/local/cuda-12.5 --cudnn_home /usr/local/cuda-12.5
# 编译完成之后还需要安装
cd build/Linux/Release
sudo make install
```

在这个过程中可能会遇到报cmake版本不对的问题，可以在这里获取到最新版本的cmake：https://github.com/Kitware/CMake/releases/

但是建议先不要更换cmake版本到3.30以上，否则可能会遇到一堆关于cmake_minimun_version的报错，很离谱。

如果不小心安装了其他版本的cmake，也没关系，重新选择所需版本的对应脚本，重新执行下面的命令就行了。

```bash
sudo bash ./cmake-3.28.6-linux-x86_64.sh --skip-license --prefix=/usr
# 上述过程完成，就是安装完成了，输入以下命令查看版本
cmake --version
```

在更新cmake之后，重新执行onnx的build脚本即可。

在后续的编译中，只需要在target_link_libraries中加上'-lonnxruntime'即可。如果遇到了报错：“error while loading shared libraries: libonnxruntime.so.1.8.0: cannot open shared object file: No such file or directory”，则需要把这个动态库路径添加到环境变量LD_LIBRARY_PATH中:

```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
```

对于onnx模型的结构，以及输入输出的name，可以在这里用netron工具查看：https://github.com/lutzroeder/netron/releases
