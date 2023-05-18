# **Dockerfile Instructions **

### **1.Dockerfile content**

##### 1.Select arm64v8/ubuntu:bionic as the base image

```Bash
#Specified base image
FROM arm64v8/ubuntu:bionic
#Specify the cmake version
ENV CMAKE_VER="3.21.2"
#Specify the github URL
ENV GITHUB_URL="github.com"
ENV GITHUB_RAW="raw.githubusercontent.com"
#Specified work path
WORKDIR home/builder
```

**2.Install and download tools**

```Bash
RUN apt-get update \
&& apt-get install -q -y --no-install-recommends wget \
  ca-certificates \
#Example Delete the index file of the software package resource
&& rm -rf /var/lib/apt/lists/*
```

**3.Change the software source list**

```Bash
RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic main restricted universe multiverse\n" > /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-updates main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-backports main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ bionic-security main restricted universe multiverse" >> /etc/apt/sources.list
```

**4.Set time**

```Bash
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/config-deb.tar.gz && \
  tar -xzvf config-deb.tar.gz && \
  dpkg -i config-deb/tzdata/*.deb && \
  rm -rf *
```

**5.Setting environment variables**

```Bash
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
```



**6.Install some feature packs**

```Bash
RUN wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/base-deb.tar.gz \
&& tar -xzvf base-deb.tar.gz \
&& cp base-deb/*.deb /var/cache/apt/archives/ \
&& apt install --no-install-recommends /var/cache/apt/archives/*.deb \
&& rm -rf *
```

**7.Modified pip**

```Bash
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple pip -U
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```



**8.Set the ros2 source list**

```Bash
RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ bionic main" > /etc/apt/sources.list.d/ros2-latest.list \
&& apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```



**9.Install ros2**

```Bash
RUN mkdir carpo-ros2-debs \
&& wget https://cnbj2m-fds.api.xiaomi.net/bsp-internal/ROS/carpo-ros2-debs/carpo-ros2-debs.tgz \
&& tar -xf carpo-ros2-debs.tgz -C carpo-ros2-debs \
&& dpkg -i carpo-ros2-debs/*.deb \
&& rm -rf *
```

**10.Create the nvidia installation environment**

```Bash
RUN mkdir -p /opt/nvidia/l4t-packages/ \
&& touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall \
&& rm /var/lib/dpkg/info/* -rf
```



**11.Install compilation environment**

```Bash
#Download the required offline packages and configuration files from the server
RUN wget https://cnbj2m.fds.api.xiaomi.com/bsp-internal/ROS/open-source-docker-depends/docker-depend.tar.gz \
#Unzip the offline package
&& tar -xzvf docker-depend.tar.gz \
#Install the deb package compiled from the source code
&& dpkg -i docker-depend/dpkg-deb/*.deb \
#Place the other deb offline packages in the apt download directory and install them
&& cp docker-depend/apt-deb/*.deb /var/cache/apt/archives/ \
&& apt install --no-install-recommends -y /var/cache/apt/archives/*.deb \
#nstall the ypthon package
&& python3 -m pip install --no-index --find-link ./docker-depend/whls/ -r ./docker-depend/whls/requirement.txt --ignore-installed \
#Install static libraries and headers
&& cp docker-depend/config-file/libwebrtc.a /usr/local/lib \
&& cp docker-depend/config-file/libgalaxy-fds-sdk-cpp.a /usr/local/lib/ \
&& cp -r docker-depend/config-file/webrtc_headers/ /usr/local/include/ \
&& cp -r docker-depend/config-file/include/* /usr/local/include/ \
&& cp -r docker-depend/config-file/grpc-archive/* /usr/local/lib/ \
&& cp docker-depend/config-file/ldconf/* /etc/ld.so.conf.d \
#Linked dynamic library
&& ldconfig \
&& rm -rf * 
```



**12.Set python3 to the system default version**

```Bash
RUN rm -f /usr/bin/python \
&& ln -s /usr/bin/python3 /usr/bin/python
```

**13.Set the environment configuration of ros2**

```Bash
RUN echo "ros2_galactic_on(){" >> /root/.bashrc && \
  echo "export ROS_VERSION=2" >> /root/.bashrc && \
  echo "export ROS_PYTHON_VERSION=3" >> /root/.bashrc && \
  echo "export ROS_DISTRO=galactic" >> /root/.bashrc && \
  echo "source /opt/ros2/galactic/setup.bash" >> /root/.bashrc && \
  echo "}" >> /root/.bashrc
```



### **2.Image making process**

**1.Create an empty folder and copy the Dockerfile file into it**

```Bash
mkdir myubuntu
cp Dockerfile myubuntu/
```

**2.Execute in the directory where the Dockerfile file resides:**

```Bash
sudo docker build -t ImageName:tag .
For example：sudo docker build -t myimage:1.0.0 .
```

### **3. docker compiles and replaces feature packs in dogs**

**1.Create a container and create a mapping between a local directory and an image directory**

```Bash
sudo docker run --privileged=true -it -v  /LocalDirectory:/DirectoryImageName imageName：tag bash
For example：sudo docker run --privileged=true -it -v /home/xxx/cyberdog_ws:/home/builder/cyberdog_ws myimage:1.0.0 bash
```

**2.Enter the cyberdog\_ws directory on docker terminal and execute：**

```Bash
source /opt/ros2/galactic/setup.bash
#To upgrade a single feature pack, simply compile package-name
colcon build --merge-install --packages-select $package-name
```

**3.Go to the cyberdog\_ws directory on the local terminal and put the compiled product in the inatall/ directory on the dog for the specific package：**

```Bash
scp -r install/lib/$package-name mi@192.168.55.1:/opt/ros2/cyberdog/lib/
scp -r install/share/$package-name mi@192.168.55.1:/opt/ros2/cyberdog/share/
scp -r install/include/$package-name mi@192.168.55.1:/opt/ros2/cyberdog/include/
```

**4.The function pack takes effect after the dog is restarted**

