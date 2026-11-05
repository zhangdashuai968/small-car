#!/home/abot/miniconda3/envs/39/bin/python
# -*- coding: utf-8 -*-
# quiz_vlm.py — 看题：拍一帧问VLM"画的是什么水果"（39环境独立脚本）
# 用法: quiz_vlm.py [超时秒]  → stdout 输出水果名（如 apple）或 FAIL
import sys, time
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from openai import OpenAI

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
client = OpenAI(api_key="sk-4f51e302d219401eb0e21dc9aa7f9bbd",
                base_url="https://dashscope.aliyuncs.com/compatible-mode/v1")

frame = {'img': None, 't': 0}
def cb(msg):
    if frame['img'] is None:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        frame['img'] = arr.copy()
        frame['t'] = time.time()

rospy.init_node('quiz_vlm', anonymous=True)
rospy.Subscriber('/new_topic', Image, cb, queue_size=1)
t0 = time.time()
while frame['img'] is None and time.time() - t0 < 5 and not rospy.is_shutdown():
    time.sleep(0.1)
if frame['img'] is None:
    print('FAIL'); sys.exit(1)

ok, buf = cv2.imencode('.jpg', frame['img'])
if not ok:
    print('FAIL'); sys.exit(1)
import base64
b64 = base64.b64encode(buf.tobytes()).decode()

try:
    resp = client.chat.completions.create(
        model="qwen-vl-max",
        messages=[{"role": "user",
                   "content": [{"type": "image_url",
                                "image_url": {"url": "data:image/jpeg;base64," + b64}},
                               {"type": "text",
                                "text": "画面中画的是什么水果？只回答一个英文单词（apple/banana/orange/pear/grape等），没有水果回答NONE"}]}],
        timeout=TIMEOUT)
    ans = resp.choices[0].message.content.strip().lower()
    for w in ['apple','banana','orange','pear','grape','peach','lemon','mango','strawberry']:
        if w in ans:
            print(w); sys.exit(0)
    print('FAIL')
except Exception as e:
    print('FAIL')
