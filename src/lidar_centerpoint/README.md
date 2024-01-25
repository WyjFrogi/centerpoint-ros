# lidar_centerpoint


- 第一版CMakeLists.txt，适配ros1


- 第二版CMakeLists.txt


- 0119,继续适配ros1,下一步需要修改msg


- 0122,修改了msg,但是后续可能还需要增加msg,引入两个新的功能包


- 0123,功能包和msg包添加完毕,单独编译没有问题,但是一起编译时tensorrt_common包会报错,找不到tensorrt的头文件

- 0123,回到家中在wsl2的ubuntu系统里。tensorrt是用deb安装至/usr/include/x86_64-linux-gnu和/usr/lib/x86_64-linux-gnu下，这次编译tensorrt_common包不报错，虽然我又重新将tar包解压至至/usr/local/下，但目前还不清楚到底用的是哪个路径下的库和头文件

- 0125,回到家中在wsl2的ubuntu系统里.修改完代码，编译通过。但是还没有找到tensorrt的头文件的错误

