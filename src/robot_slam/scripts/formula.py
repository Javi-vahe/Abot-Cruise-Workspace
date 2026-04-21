#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int16
import requests
import codecs
import json
import base64
import time
import re  # 导入正则表达式模块
from cv_bridge import CvBridge
import cv2

class ImageApiSender:
    def __init__(self):
        # 替换为你的 API 密钥
        self.API_KEY = 'xxx'
        # 设置请求的 URL
        self.url = 'xxx'
        # 创建 CvBridge 对象
        self.bridge = CvBridge()
        # 全局计数器
        self.json_counter = 1
        self.send_req = False
        self.send_times = 0
        self.receive_req_time = time.time()

        # self.last_image = Image()

        # 初始化 ROS 节点
        rospy.init_node('image_api_sender', anonymous=False)
        # 订阅图像话题
        self.sub_image = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.sub_req = rospy.Subscriber("/req_api", Empty, self.req_api_callback)
        self.pub_id = rospy.Publisher("/formula_id", Int16, queue_size=1)

    def extract_last_number(self, content):
        # 使用正则表达式提取最后一个数字
        match = re.findall(r'\d+', content)  # 匹配所有数字
        if match:  # 如果找到数字
            return match[-1]  # 返回最后一个数字
        return None  # 如果没有找到数字，返回 None

    def req_api_callback(self, msg):
        self.send_times += 1
        rospy.logwarn("received %d times req" % self.send_times)
        self.receive_req_time = time.time()
        self.send_req = True

    def image_callback(self, msg):
        # self.last_image = msg
        if not self.send_req:
            return
        # 收到请求立即置为false
        self.send_req = False

        # 将 ROS 图像消息转换为 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 将 OpenCV 图像编码为 PNG 格式
        _, buffer = cv2.imencode('.png', cv_image)

        # 将图像转换为 base64 编码
        img_base64 = base64.b64encode(buffer).decode('utf-8')

        # 设置请求数据
        description = "Please directly answer the numerical result of the math problem in the image! Note: Limit your response to 5 characters or less, and the correct answer must be a number between 1 and 8. If not, you have calculated incorrectly!!!Note that your calculation should conform to the rules of addition and subtraction!!!"  # 你的简单描述
        data = {
            "model": "yi-vision",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": "data:image/png;base64,%s" % img_base64  
                            }
                        },
                        {
                            "type": "text",
                            "text": description
                        }
                    ]
                }
            ],
            "stream": False,
            "max_tokens": 1024
        }

        # 设置请求头
        headers = {
            'Authorization': 'Bearer %s' % self.API_KEY,
            'Content-Type': 'application/json',
        }

        # 发送 POST 请求
        rospy.logwarn("sending req...")
        start_time = time.time()
        response = requests.post(self.url, headers=headers, data=json.dumps(data))

        # 检查响应状态码
        if response.status_code == 200:
            result = response.json()  # 获取 JSON 数据
            print(result)

            # 获取 content
            content = result["choices"][0]["message"]["content"]
            print("Response content:", content)

            # 提取最后一个数字
            last_number = self.extract_last_number(content)
            if last_number is not None:
                print("Formula result:", last_number)
                msg_to_pub = Int16()
                msg_to_pub.data = int(last_number)
                self.pub_id.publish(msg_to_pub)
            else:
                print("no result!!!!!")
            print("api time cost:", time.time() - start_time)
            print("receive_to_pub time cost:", time.time() - self.receive_req_time)

            # 将 JSON 数据保存为文件
            json_filename = 'result_%d.json' % self.json_counter  # 使用计数器命名文件
            with codecs.open(json_filename, 'w', encoding='utf-8') as json_file:  # 使用 codecs 处理编码
                json.dump(result, json_file, ensure_ascii=False, indent=4)  # 保存为 JSON 文件

            self.json_counter += 1  # 递增计数器
        else:
            rospy.logerr("request failure, status code: %d, content: %s" % (response.status_code, response.text))

    def run(self):
        # 循环等待回调
        rospy.spin()

if __name__ == '__main__':
    image_api_sender = ImageApiSender()
    image_api_sender.run()
