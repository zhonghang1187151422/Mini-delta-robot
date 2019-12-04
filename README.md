基于INNFOS驱动器的Delta机器人
=
这是一个采用INNFOS公司3个QDD Lite-NE30-36和1个QDD Lite-EL20-36执行器所组成的4关节Delta型机器人的驱动包。执行器的驱动包是从</html>https://github.com/innfos/innfos-cpp-sdk.git</html>上fock过来。Delta机器人的驱动包为deltarobot文件夹中，其主要包括以下几个功能： <br>
（1）实现了Delta型机器人的正逆运动学及其速度空间变换，见DeltaRobotKinematics类 <br>
（2）实现了关节空间和直线轨迹的运动规划，能够设置加减速因子，见DeltaRobotMotionPlanning类 <br>
（3）实现了Delta机器人的位置控制和速度控制，见DeltaRobotControl类 <br>

## 机器人配置
机器人由3个QDD Lite-NE30-36电机和1个QDD Lite-EL20-36电机组成。3个QDD Lite-NE30-36电机安装在基座上呈120度排列，其电机ID分别为11、12、13。QDD Lite-EL20-36电机安装在末端，其ID为14。

## 环境依赖
LINUX Ubuntu16.04

## 安装编译
（1）将软件包下载到本地 <br>
```Bash
git clone https://github.com/zhonghang1187151422/Mini-delta-robot.git
```
（2）在文件夹下打开终端，进入example目录，该目录下有CMakeLists.txt
```Bash
cd ./example
```
（3）在该路径下新建一个build文件夹，并切换路径到此文件夹
```Bash
mkdir ./build
cd ./build
```
（3）编译代码
```Bash
cmake ..
make
```

编译完成后会在build文件夹下生成一个bin文件夹，下面就是示例代码的可执行文件。

## 测试
提供两个关于Delta机器人的测试可执行文件，deltarobot_posctl_test1和deltarobot_velctl_test1，一个是位置控制，一个是速度控制。
（1）切换到bin文件夹下并
```Bash
cd ./bin
```
（2）运行程序
比如要测试Delta机器人的位置控制功能，运行为：
```Bash
./deltarobot_posctl_test1
```

