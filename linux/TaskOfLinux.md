# Linux 任务

在虚拟机或实体机上安装`Ubuntu`，磁盘选择手动分区，不设置`Swap`分区，使用`btrfs`文件系统，使用`zram`作为`Swap`

https://www.myfreax.com/how-to-configure-zram-on-ubuntu-20-04/

写一个`shell`脚步，实现：
运行脚本时创建一个根分区的快照，保存在`/.snapshot`下，快照名为当天的时间，如`2023-09-13`，如果当天的快照已经存在，则不创建快照。
