首先显卡驱动得有，在 Terminal 里面输入 `nvidia-smi` 确认显卡驱动是否正常。如果有如下输出则是正常的。
![nvidia-smi](illustration/nvidia-smi.png)

```bash
git clone --recursive https://github.com/Microsoft/onnxruntime
cd onnxruntime/
# not use cuda
./build.sh --skip_tests --config Release --build_shared_lib --parallel
# use cuda
./build.sh --skip_tests --use_cuda --config Release --build_shared_lib --parallel --cuda_home /usr/local/cuda-12.5 --cudnn_home /usr/local/cuda-12.5
```

在这个过程中可能会遇到报cmake版本不对的问题，可以在这里获取到最新版本的cmake：https://github.com/Kitware/CMake/releases/

```bash
sudo bash ./cmake-4.0.3-linux-x86_64.sh --skip-license --prefix=/usr
# 上述过程完成，就是安装完成了，输入以下命令查看版本
cmake --version
```

在更新cmake之后，重新执行onnx的build脚本，第一次还是会报错，再执行一次就可以了。
