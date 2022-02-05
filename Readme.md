# SensorFusion

异步融合激光雷达与毫米波雷达的检测结果，数据与代码实现参考知乎教程练习的一个小demo


### 目录结构

```
多传感器融合的代码目录结构如下所示：
|___SensorFusion
    |___bin 代码编译后的可执行文件存放目录
    |___build 代码编译时的过程文件存放处
    |___data 传感器融合的输入数据
    |___src 传感器融合的源代码
    |___build.sh 编译脚本
    |___CMakeLists.txt 组建工程的CMake文件
    |___LICENSE Udacity的License
    |___Reame.md 使用说明
```

### 使用方法

系统
Linux Ubuntu 18.04
下载Eigen库放在src/文件夹中

编译与运行

```shell
mkdir -p build
cd build
cmake ..
make
cd ../bin
./SensorFusion ../data/lidar_raw_data.txt
./SensorFusion ../data/radar_raw_data.txt
./SensorFusion ../data/sample-laser-radar-measurement-data-2.txt
```

参考资料：
卡尔曼滤波
https://zhuanlan.zhihu.com/p/45238681
拓展卡尔曼滤波
https://zhuanlan.zhihu.com/p/63641680
传感器融合
https://zhuanlan.zhihu.com/p/67139241
存在的问题：
1. 从传感器数据时间戳来看，这个demo采用的是异步融合，相当于两个传感器交替更新，起到了提高采样频率的效果，不是真正意义上的同步融合改善检测精度
2. 与真正的传感器检测结果差别较大，仿真数据将物体视作一个质点，真实的传感器需要对点云进行处理得到检测结果
3. 先验信息的获取，比如状态协方差矩阵与过程噪声等，实际工程应用中较难获取
4. 车辆运动模型过于简化