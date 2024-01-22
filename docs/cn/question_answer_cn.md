开发者QA问题汇总

## 刷机包在哪里下载？

```Bash
【CyberDog2 刷机包下载】https://s.xiaomi.cn/c/8JEpGDY8
```

## 雷达自检失败是什么原因？

```Bash
更新英伟达官方软件包，可能导致雷达自检失败
1.目前已知nvidia自带的服务nvgetty.service会使用和雷达同样的串口，两者会产生冲突，我们在出厂时已经将该服务禁用掉了。
2.可以使用如下命令查看下是否该服务在运行：
sudo systemctl status nvgetty.service
3.如果在运行，使用如下命令禁用掉：
sudo systemctl disable nvgetty.service
执行之后重启即可
```

## 编译过不去，遇到各种问题怎么办？

```Plain
1,进入tools目录下，可使用Dockerfile文件编译镜像
2，参考Dockerfile文档 https://github.com/MiRoboticsLab/blogs/blob/rolling/docs/cn/dockerfile_instructions_cn.md 进行编译，如果还有其它问题，可以在issue中提问。
```

## cyberdog与cyberdog2对应仓库

```Bash
cyberdog 对应仓库地址：https://github.com/MiRoboticsLab/cyberdog_ros2
cyberdog2对应仓库地址：https://github.com/MiRoboticsLab/cyberdog_ws
```
