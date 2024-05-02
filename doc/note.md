fast-livo 是多线程，在tree重建的时候

在ros工程中,在while(ros::ok())外面已经做好回调函数，在里面实现对IMU、激光雷达点云、可见光图像处理，其中IMU进行前向传播和反向传播，在这个过程中纠正点云，然后对图像求解重投影误差和最小化广度误差，最后将点云与图像融合。

修改 debug(原本 =0 ) 和 fast_lio_is_ready（原本 = false ）

int debug = 1;
bool fast_lio_is_ready = true;

launch-prefix="xterm -geometry 80x24 -fa 'Monospace' -fs 12  -e gdb -ex run --args"

DON'T USE THIS CODE！！！