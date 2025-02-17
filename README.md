# Localization Practice!  
[실습환경: ubuntu 20.04 및 ros-noetic]  
**실습하실 분들은 위의 환경 구현을 먼저 완료하신 후 아래 절차를 따르시면 됩니다.  
현재 이미 다른 버전의 우분투나 ros를 사용중이시라 위의 환경 구현이 어렵다면 Docker를 이용한 가상환경 구축을 권장드립니다.**   

## 1️⃣ 필수 패키지 설치
```bash
sudo apt update && sudo apt install -y git-lfs  
git lfs install
sudo apt install ros-noetic-ublox
```


## 2️⃣ 워크스페이스 구축
```bash
mkdir -p ~/catkin_ws/src && cd catkin_ws/src  
catkin_init_workspace
cd ..
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## 3️⃣ 프로젝트 클론 및 의존성 설치
```bash
cd ~/catkin_ws/src  
git clone https://github.com/hee18/Localization_practice.git     
cd ..    
catkin_make  
source devel/setup.bash
cd src/Localization_practice
pip3 install -r requirements.txt
```


## 4️⃣ 측위 프로젝트 실습 (터미널 창 4개 띄워놓고 진행)
```bash
# 1st window
roscore


# 2nd window
# if rosbag play 오류 발생 => convert_CAN_bag.py 실행시켜서 다른 이름으로 bagfile 저장하고 다시 play
cd ~/catkin_ws/src/Localization_practice/localization/bagfiles
rosbag play practice_data.bag


# 3rd window
cd ~/catkin_ws/src/Localization_practice/localization/codes
python3 localization_hackers.py


# 4th window
cd ~/catkin_ws/src/Localization_practice/localization/codes
python3 localization.py
```
