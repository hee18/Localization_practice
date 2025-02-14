import rosbag
from localization_practice.msg import CANOutput  # 현재 ROS 환경에서 사용하는 메시지 타입
from std_msgs.msg import String, Header

input_bag = 'reduced_data.bag'
output_bag = 'practice_data.bag'

with rosbag.Bag(output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if topic == "/CANOutput":
            try:
                new_msg = CANOutput()
                
                # 기존 header 유지
                new_msg.header = msg.header if hasattr(msg, "header") else Header()

                # StrAng(조향각)과 VS(차량 속도)만 유지
                new_msg.StrAng = msg.StrAng if hasattr(msg, "StrAng") else String(data="0")
                new_msg.VS = msg.VS if hasattr(msg, "VS") else String(data="0")

                outbag.write(topic, new_msg, t)

            except Exception as e:
                print(f"[Error] 메시지 변환 실패 (시간: {t}): {e}")

        else:
            outbag.write(topic, msg, t)

print("rosbag 변환 완료! /CANOutput에서 StrAng, VS만 유지됩니다.")
