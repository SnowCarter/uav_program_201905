
运行很简单：
先运行含有 点云 的终端
运行：含有tf信息 的终端
运行：filterkalman 的launch 文件

如果有blam 文件:修改bag文件的路径为自己需要运行的bag文件
然后：
运行blam的launch文件，然后运行filterkalman 的launch文件。

打开rviz


编译：
修改程序，先用qt编译一下，没有错误。

建议在修改完之后，运行前 在catkin_ws 目录下执行 ： catkin_make  再编译一次。

