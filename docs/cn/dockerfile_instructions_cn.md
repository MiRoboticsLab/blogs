# **Dockerfile使用说明**

### **一、Dockerfile内容**

##### 1.选择arm64v8/ubuntu:bionic为基础镜像

```Bash
#指定基础镜像
FROM arm64v8/ubuntu:bionic
#指定cmake版本
ENV CMAKE_VER="3.21.2"
#指定github网址
ENV GITHUB_URL="github.com"
ENV GITHUB_RAW="raw.githubusercontent.com"
#指定工作路径
WORKDIR home/builder
```

**2.安装下载工具**

```Bash
RUN apt-get update \
&& apt-get install -q -y --no-install-recommends wget \
  ca-certificates \
#删除软件包资源索引文件
&& rm -rf /var/lib/apt/lists/*
```

**3.更换软件源列表**

```Bash
RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse\n" > /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse" >> /etc/apt/sources.list
```

**4.设置时间**

```Bash
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/config-deb.tar.gz && \
  tar -xzvf config-deb.tar.gz && \
  dpkg -i config-deb/tzdata/*.deb && \
  rm -rf *
```

**5.设置环境变量**

```Bash
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
```



**6.安装一些功能包**

```Bash
RUN wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/base-deb.tar.gz \
&& tar -xzvf base-deb.tar.gz \
&& cp base-deb/*.deb /var/cache/apt/archives/
RUN echo y| apt-get install --no-install-recommends /var/cache/apt/archives/*.deb && rm -rf *
```

**7.修改pip**

```Bash
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple pip -U
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```



**8.设置ros2源列表**

```Bash
RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ bionic main" > /etc/apt/sources.list.d/ros2-latest.list \
&& apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```



**9.安装ros2**

```Bash
RUN mkdir carpo-ros2-debs \
&& wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/carpo-ros2-debs/carpo-ros2-debs.tgz \
&& tar -xf carpo-ros2-debs.tgz -C carpo-ros2-debs \
&& dpkg -i carpo-ros2-debs/*.deb \
&& rm -rf *
```

**10.创建nvidia安装环境**

```Bash
RUN mkdir -p /opt/nvidia/l4t-packages/ \
&& touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall \
&& rm /var/lib/dpkg/info/* -rf
```



**11.安装编译环境**

```Bash
#从服务器上下载所需要的离线包和配置文件
RUN wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/docker-depend.tar.gz \
#解压离线包
&& tar -xzvf docker-depend.tar.gz \
#安装通过源码编译得到的deb包
&& dpkg -i docker-depend/dpkg-deb/*.deb \
#将其它deb离线包放在apt的下载目录并安装
&& cp docker-depend/apt-deb/*.deb /var/cache/apt/archives/ \
&& apt install --no-install-recommends -y /var/cache/apt/archives/*.deb \
#安装ypthon包
&& python3 -m pip install --no-index --find-link ./docker-depend/whls/ -r ./docker-depend/whls/requirement.txt --ignore-installed \
#安装静态库和头文件
&& cp docker-depend/config-file/libwebrtc.a /usr/local/lib \
&& cp docker-depend/config-file/libgalaxy-fds-sdk-cpp.a /usr/local/lib/ \
&& cp -r docker-depend/config-file/webrtc_headers/ /usr/local/include/ \
&& cp -r docker-depend/config-file/include/* /usr/local/include/ \
&& cp -r docker-depend/config-file/grpc-archive/* /usr/local/lib/ \
&& cp docker-depend/config-file/ldconf/* /etc/ld.so.conf.d \
#链接动态库
&& ldconfig \
&& rm -rf * 
```



**12.设置python3为系统默认版本**

```Bash
RUN rm -f /usr/bin/python \
&& ln -s /usr/bin/python3 /usr/bin/python
```

**13.设置ros2的环境配置**

```Bash
RUN echo "ros2_galactic_on(){" >> /root/.bashrc && \
  echo "export ROS_VERSION=2" >> /root/.bashrc && \
  echo "export ROS_PYTHON_VERSION=3" >> /root/.bashrc && \
  echo "export ROS_DISTRO=galactic" >> /root/.bashrc && \
  echo "source /opt/ros2/galactic/setup.bash" >> /root/.bashrc && \
  echo "}" >> /root/.bashrc
```



### **二、镜像制作流程**

**1.本机安装好docker环境**

**2.我们的电脑架构通常是x86架构，如果直接运行arm架构的镜像会报错，需要执行下面的命令：**

```Bash
sudo apt install -y qemu-user-static binfmt-support
docker run --privileged multiarch/qemu-user-static --reset -p yes
```

**3.新建一个空文件夹，将Dockerfile文件复制到这个空目录中**

```Bash
mkdir myubuntu
cp Dockerfile myubuntu/
```

**4.在 Dockerfile 文件所在目录执行：**

```Bash
sudo docker build -t 镜像名:tag .
例如：sudo docker build -t myimage:1.0.0 .
```

### **三、 docker 编译并替换狗里面的功能包**

**1.创建容器并建立本地目录到镜像目录的映射关系**

```Bash
sudo docker run --privileged=true -it -v /本地目录:/镜像中的目录 镜像名：tag bash
#这里xxx代表你的用户名
例如：sudo docker run --privileged=true -it -v /home/xxx/cyberdog_ws:/home/builder/cyberdog_ws myimage:1.0.0 bash
```

**2.在docker终端进入到cyberdog\_ws目录下执行：**

```Bash
source /opt/ros2/galactic/setup.bash
#第一次编译某个功能包需要使用--packages-up-to,编译该功能包及其依赖包
colcon build --merge-install --packages-up-to $package-name
#后续升级单个功能包使用--packages-select，只编译该功能包
colcon build --merge-install --packages-select $package-name
```

**3.开启一个终端，电脑和狗用数据线连接，输入如下命令进入到狗上，密码为123**

```Bash
ssh mi@192.168.55.1
```

**4.对于具体的包来说执行下面命令将编译产物放入到狗上相应位置** 

```Bash
#本地终端cyberdog_ws目录
scp -r install/lib/$package-name mi@192.168.55.1:/home/mi
#狗上终端/home/mi目录
cp -rf $package-name /opt/ros2/cyberdog/lib
sudo rm -rf $package-name

#本地终端cyberdog_ws目录
scp -r install/share/$package-name mi@192.168.55.1:/home/mi
#狗上终端/home/mi目录
cp -rf $package-name /opt/ros2/cyberdog/share
sudo rm -rf $package-name

#本地终端cyberdog_ws目录
scp -r install/include/$package-name mi@192.168.55.1:/home/mi
#狗上终端/home/mi目录
cp -rf $package-name /opt/ros2/cyberdog/include
sudo rm -rf $package-name
```

**5.狗重启后功能包生效。**

