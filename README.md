基于INNFOS驱动器的Delta机器人
=
这是一个采用INNFOS公司3个QDD Lite-NE30-36和1个QDD Lite-EL20-36执行器所组成的4关节Delta型机器人的驱动包。执行器的驱动包是从</html>https://github.com/innfos/innfos-cpp-sdk.git</html>上fock过来。Delta机器人的驱动包为deltarobot文件夹中，其主要包括以下几个功能： <br>
（1）实现了Delta型机器人的正逆运动学及其速度空间变换，见DeltaRobotKinematics类 <br>
（2）实现了关节空间和直线轨迹的运动规划，能够设置加减速因子，见DeltaRobotMotionPlanning类 <br>
（3）实现了Delta机器人的位置控制和速度控制，见DeltaRobotControl类 <br>

## 环境依赖
linux ubuntu16.04

## 安装使用
（1）将软件包下载到本地 <br>
```Bash
git clone https://github.com/zhonghang1187151422/Mini-delta-robot.git
```
（2）在文件夹下打开终端，进入example目录，该目录下有CMakeLists.txt
```Bash
cd ./example
```
（3）编译代码
```Bash
cmake CMakeLists.txt
make
```
输入命令执行完成后，在该目录下会生成一个bin文件夹，该目录存放了生成的示例程序。
（4）测试程序
先进入bin文件夹：
```Bash
cd ./bin
```
比如要测试Delta机器人的位置控制功能，运行为：
```Bash
./deltarobot_posctl_test1
```

