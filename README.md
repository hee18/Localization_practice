# Localization Practice!
## 1️⃣ 필수 패키지 설치
```bash
sudo apt update && sudo apt install -y git-lfs  
git lfs install
sudo apt install ros-noetic-ublox
```

## 2️⃣ 프로젝트 클론 및 의존성 설치
```bash
cd /catkin_ws/src  
git clone https://github.com/n-submarine/Localization_practice.git  
git clone https://github.com/KumarRobotics/ublox.git    
cd ..  
rosdep install --from-paths src --ignore-src -r -y  
catkin_make  
source devel/setup.bash
```
