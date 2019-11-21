트램 센서 드라이버
===================================

ubuntu 패캐지 업데이트:

    sudo apt-get update  

1.Kvaser Can 드라이버
-----------------------------------
Kvaser Can을 사용하기 위한 sdk & 드라이버

### Installing ###

Download Kvaser Linux Driver and SDK:

    https://www.kvaser.com/download/

홈페이지 에서 "Kvaser Linux Drivers and SDK" 다운로드

드라이버 설치:

    cd ~/Downloads
    sudo mv linuxcan.tar.gz /usr/src
    cd /usr/src
    sudo rm -rf linuxcan/
    sudo tar xvf linuxcan.tar.gz
    cd linuxcan/
    make
    sudo make install

리눅스 Kvaser CAN 드라이버 설치

설치 확인:

    cd /usr/src/linuxcan/canlib/examples
    ./listChannels    

Kvaser CAN 채널 정보가 출력이 되면 정상적으로 설치가 완료가 된것이다.


Kvaser 인터페이스 드라이버 설치:    

    $sudo apt update && sudo apt install apt-transport-https
    $sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
    $sudo apt-add-repository ppa:jwhitleyastuff/kvaser-linux
    $sudo apt-get update
    $sudo apt install ros-$ROS_DISTRO-kvaser-interface

ROS에서 지원하는 Kvaser CAN 인터페이스 노드를 설치힌다.


2.Cemara센서 드라이버
-----------------------------------
위키 참고

    http://wiki.ros.org/pylon_camera

### Installing ###

SDK 다운로드:

    https://www.baslerweb.com/de/support/downloads/downloads-software/    

위 사이트 에서 "pylon 5.2.0 Camera Software Suite Linux x86 (64 Bit) - Debian Installer Package" 다운로드

SDK 설치:

    $ cd ~/Downloads/
    $ sudo dpkg -i pylon_5.2.0.13457-deb0_amd64.deb

ROS 패키지 의존성 구성:

    $ sudo sh -c 'echo "yaml https://raw.githubusercontent.com/magazino/pylon_camera/indigo-devel/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list'
    $ rosdep update


source 다운 및 빌드:    

    $ cd ~/catkin_ws/src/ && git clone https://github.com/magazino/pylon_camera.git && git clone https://github.com/magazino/camera_control_msgs.git
    $ rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y
    $ cd ~/catkin_ws && catkin_make

### pylon camera 노드 설정 ###
설정 파일은 수정하여 인코딩 값을 변경해야 한다.

 * `설정 파일 위치`: roscd pylon_camera/config/

default.yaml:

    image_encoding: "bayer_bggr8"

image_encoding 값을 "bayer_bggr8" 로 수정해야 한다.

3.ROS 센서 드라이버
-----------------------------------

### Installing ###

source 다운 및 빌드:    

    cd ~/catkin_ws/src/ && git clone https://github.com/YunGilDong/ros_sensor_driver_.git    
    cd ~/catkin_ws && catkin_make
