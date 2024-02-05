# lidar_centerpoint


- 第一版CMakeLists.txt，适配ros1


- 第二版CMakeLists.txt


- 0119,继续适配ros1,下一步需要修改msg


- 0122,修改了msg,但是后续可能还需要增加msg,引入两个新的功能包


- 0123,功能包和msg包添加完毕,单独编译没有问题,但是一起编译时tensorrt_common包会报错,找不到tensorrt的头文件

- 0123,回到家中在wsl2的ubuntu系统里。tensorrt是用deb安装至/usr/include/x86_64-linux-gnu和/usr/lib/x86_64-linux-gnu下，这次编译tensorrt_common包不报错，虽然我又重新将tar包解压至至/usr/local/下，但目前还不清楚到底用的是哪个路径下的库和头文件

- 0125,回到家中在wsl2的ubuntu系统里.修改完代码，编译通过。但是还没有找到tensorrt的头文件的错误

- 0126,是lidar_centerpoint包里面也引用了tensorrt头文件，但是没有指定头文件的路径，指定路径后，最终编译通过，后续工作需要继续修改代码，完善功能

- 0129,加入了点云预处理,去地面

- 0205,修复了tensorRT推理变量析构顺序不正确的错误，用bag包能将代码跑起来，但是由于没有正确的坐标变换，只能跑到预处理，检测模块没跑起来，所以后续需要加入正确的tf关系
