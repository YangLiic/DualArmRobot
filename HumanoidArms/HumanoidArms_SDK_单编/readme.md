### 该操作库函数的所有输入输出参数，均采用国际单位,即长度（m），线速度（m/s ），角度（rad），角速度（rad/s），时间（s）

1.运动        
moveJ_ToJoint  关节运动到目标关节角              
moveJ_ToPos   关节运动到笛卡尔位置  xyzrxryrz               
moveL_ToPos   线性运动到目标位置           

2.can_init();  初始化can设备      
    Exit();  退出can设备       
这两个函数是成对出现的，否则会有设备被占用的问题

3.src文件夹中是示例程序

4.inclue文件夹下的Ti5BASIC.h是所有功能函数，里面有详细说明，使用前请先阅读

5.C++依赖库
    libmylibti5.so，libcontrolcan.so
    需要将这两个库cp到/usr/lib下，或者每次编译和执行的时候指定绝对路径

6.C++编译命令
    export CPLUS_INCLUDE_PATH=/你的文件夹路径/include:$CPLUS_INCLUDE_PATH
    g++ main.cpp -I/usr/include/eigen3 -L../usrlib/ -lmylibti5 -lcontrolcan -lncurses -o move_sov
    或者用Makefile来编译

7.执行
    sudo ./move_sov
    