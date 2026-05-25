#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy

import cv2 as cv
import numpy as np
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from utils import CvFpsCalc


class FaceMesh:
    def __init__(self):
        image_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')

        self.getImageStatus = False

        self.max_num_faces = 1
        self.refine_landmarks = True
        self.min_detection_confidence = 0.7
        self.min_tracking_confidence = 0.5

        self.use_brect = True

        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=self.max_num_faces,
            refine_landmarks=self.refine_landmarks,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )


        self.color_image = Image()
        self.cvFpsCalc = CvFpsCalc(buffer_len=10)

        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher('/rikibot/detection_image',  Image, queue_size=1)


        # if no image messages
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        display_fps = self.cvFpsCalc.get()
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.color_image = cv.cvtColor(self.color_image, cv.COLOR_BGR2RGB)

        debug_image = copy.deepcopy(self.color_image)
        results = self.face_mesh.process(self.color_image)
        if results.multi_face_landmarks is not None:
            for face_landmarks in results.multi_face_landmarks:
                # 外接矩形の計算
                brect = self.calc_bounding_rect(debug_image, face_landmarks)
                # 虹彩の外接円の計算
                left_eye, right_eye = None, None
                if self.refine_landmarks:
                    left_eye, right_eye = self.calc_iris_min_enc_losingCircle(
                        debug_image,
                        face_landmarks,
                    )
                # 描画
                debug_image = self.draw_landmarks(
                    debug_image,
                    face_landmarks,
                    self.refine_landmarks,
                    left_eye,
                    right_eye,
                )
                debug_image = self.draw_bounding_rect(self.use_brect, debug_image, brect)

        cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)

        self.publish_image(debug_image, image.height, image.width)

        cv.imshow('MediaPipe Face Detection Demo', debug_image)
        key = cv.waitKey(1)

    def calc_bounding_rect(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_array = np.empty((0, 2), int)

        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)

            landmark_point = [np.array((landmark_x, landmark_y))]

            landmark_array = np.append(landmark_array, landmark_point, axis=0)

        x, y, w, h = cv.boundingRect(landmark_array)

        return [x, y, x + w, y + h]

    def calc_min_enc_losingCircle(self, landmark_list):
        center, radius = cv.minEnclosingCircle(np.array(landmark_list))
        center = (int(center[0]), int(center[1]))
        radius = int(radius)

        return center, radius



    def calc_iris_min_enc_losingCircle(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_point = []

        for index, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)

            landmark_point.append((landmark_x, landmark_y))

        left_eye_points = [
            landmark_point[468],
            landmark_point[469],
            landmark_point[470],
            landmark_point[471],
            landmark_point[472],
        ]
        right_eye_points = [
            landmark_point[473],
            landmark_point[474],
            landmark_point[475],
            landmark_point[476],
            landmark_point[477],
        ]

        left_eye_info = self.calc_min_enc_losingCircle(left_eye_points)
        right_eye_info = self.calc_min_enc_losingCircle(right_eye_points)

        return left_eye_info, right_eye_info

    def draw_landmarks(self, image, landmarks, refine_landmarks, left_eye, right_eye):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_point = []

        for index, landmark in enumerate(landmarks.landmark):
            if landmark.visibility < 0 or landmark.presence < 0:
                continue

            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            # landmark_z = landmark.z

            landmark_point.append((landmark_x, landmark_y))

            cv.circle(image, (landmark_x, landmark_y), 1, (0, 255, 0), 1)

        if len(landmark_point) > 0:
            # 参考：https://github.com/tensorflow/tfjs-models/blob/master/facemesh/mesh_map.jpg

            # 左眉毛(55：内側、46：外側)
            cv.line(image, landmark_point[55], landmark_point[65], (0, 255, 0), 2)
            cv.line(image, landmark_point[65], landmark_point[52], (0, 255, 0), 2)
            cv.line(image, landmark_point[52], landmark_point[53], (0, 255, 0), 2)
            cv.line(image, landmark_point[53], landmark_point[46], (0, 255, 0), 2)

            # 右眉毛(285：内側、276：外側)
            cv.line(image, landmark_point[285], landmark_point[295], (0, 255, 0), 2)
            cv.line(image, landmark_point[295], landmark_point[282], (0, 255, 0), 2)
            cv.line(image, landmark_point[282], landmark_point[283], (0, 255, 0), 2)
            cv.line(image, landmark_point[283], landmark_point[276], (0, 255, 0), 2)

            # 左目 (133：目頭、246：目尻)
            cv.line(image, landmark_point[133], landmark_point[173], (0, 255, 0), 2)
            cv.line(image, landmark_point[173], landmark_point[157], (0, 255, 0), 2)
            cv.line(image, landmark_point[157], landmark_point[158], (0, 255, 0), 2)
            cv.line(image, landmark_point[158], landmark_point[159], (0, 255, 0), 2)
            cv.line(image, landmark_point[159], landmark_point[160], (0, 255, 0), 2)
            cv.line(image, landmark_point[160], landmark_point[161], (0, 255, 0), 2)
            cv.line(image, landmark_point[161], landmark_point[246], (0, 255, 0), 2)
            cv.line(image, landmark_point[246], landmark_point[163], (0, 255, 0), 2)
            cv.line(image, landmark_point[163], landmark_point[144], (0, 255, 0), 2)
            cv.line(image, landmark_point[144], landmark_point[145], (0, 255, 0), 2)
            cv.line(image, landmark_point[145], landmark_point[153], (0, 255, 0), 2)
            cv.line(image, landmark_point[153], landmark_point[154], (0, 255, 0), 2)
            cv.line(image, landmark_point[154], landmark_point[155], (0, 255, 0), 2)
            cv.line(image, landmark_point[155], landmark_point[133], (0, 255, 0), 2)

             # 右目 (362：目頭、466：目尻)
            cv.line(image, landmark_point[362], landmark_point[398], (0, 255, 0), 2)
            cv.line(image, landmark_point[398], landmark_point[384], (0, 255, 0), 2)
            cv.line(image, landmark_point[384], landmark_point[385], (0, 255, 0), 2)
            cv.line(image, landmark_point[385], landmark_point[386], (0, 255, 0), 2)
            cv.line(image, landmark_point[386], landmark_point[387], (0, 255, 0), 2)
            cv.line(image, landmark_point[387], landmark_point[388], (0, 255, 0), 2)
            cv.line(image, landmark_point[388], landmark_point[466], (0, 255, 0), 2)
            cv.line(image, landmark_point[466], landmark_point[390], (0, 255, 0), 2)
            cv.line(image, landmark_point[390], landmark_point[373], (0, 255, 0), 2)
            cv.line(image, landmark_point[373], landmark_point[374], (0, 255, 0), 2)
            cv.line(image, landmark_point[374], landmark_point[380], (0, 255, 0), 2)
            cv.line(image, landmark_point[380], landmark_point[381], (0, 255, 0), 2)
            cv.line(image, landmark_point[381], landmark_point[382], (0, 255, 0), 2)
            cv.line(image, landmark_point[382], landmark_point[362], (0, 255, 0), 2)
             # 口 (308：右端、78：左端)
            cv.line(image, landmark_point[308], landmark_point[415], (0, 255, 0), 2)
            cv.line(image, landmark_point[415], landmark_point[310], (0, 255, 0), 2)
            cv.line(image, landmark_point[310], landmark_point[311], (0, 255, 0), 2)
            cv.line(image, landmark_point[311], landmark_point[312], (0, 255, 0), 2)
            cv.line(image, landmark_point[312], landmark_point[13], (0, 255, 0), 2)
            cv.line(image, landmark_point[13], landmark_point[82], (0, 255, 0), 2)
            cv.line(image, landmark_point[82], landmark_point[81], (0, 255, 0), 2)
            cv.line(image, landmark_point[81], landmark_point[80], (0, 255, 0), 2)
            cv.line(image, landmark_point[80], landmark_point[191], (0, 255, 0), 2)
            cv.line(image, landmark_point[191], landmark_point[78], (0, 255, 0), 2)
            cv.line(image, landmark_point[78], landmark_point[95], (0, 255, 0), 2)
            cv.line(image, landmark_point[95], landmark_point[88], (0, 255, 0), 2)
            cv.line(image, landmark_point[88], landmark_point[178], (0, 255, 0), 2)
            cv.line(image, landmark_point[178], landmark_point[87], (0, 255, 0), 2)
            cv.line(image, landmark_point[87], landmark_point[14], (0, 255, 0), 2)
            cv.line(image, landmark_point[14], landmark_point[317], (0, 255, 0), 2)
            cv.line(image, landmark_point[317], landmark_point[402], (0, 255, 0), 2)
            cv.line(image, landmark_point[402], landmark_point[318], (0, 255, 0), 2)
            cv.line(image, landmark_point[318], landmark_point[324], (0, 255, 0), 2)
            cv.line(image, landmark_point[324], landmark_point[308], (0, 255, 0), 2)

            if refine_landmarks:
            # 虹彩：外接円
                cv.circle(image, left_eye[0], left_eye[1], (0, 255, 0), 2)
                cv.circle(image, right_eye[0], right_eye[1], (0, 255, 0), 2)

                # 左目：中心
                cv.circle(image, landmark_point[468], 2, (0, 0, 255), -1)
                # 左目：目頭側
                cv.circle(image, landmark_point[469], 2, (0, 0, 255), -1)
                # 左目：上側
                cv.circle(image, landmark_point[470], 2, (0, 0, 255), -1)
                # 左目：目尻側
                cv.circle(image, landmark_point[471], 2, (0, 0, 255), -1)
                # 左目：下側
                cv.circle(image, landmark_point[472], 2, (0, 0, 255), -1)
                # 右目：中心
                cv.circle(image, landmark_point[473], 2, (0, 0, 255), -1)
                # 右目：目尻側
                cv.circle(image, landmark_point[474], 2, (0, 0, 255), -1)
                # 右目：上側
                cv.circle(image, landmark_point[475], 2, (0, 0, 255), -1)
                # 右目：目頭側
                cv.circle(image, landmark_point[476], 2, (0, 0, 255), -1)
                # 右目：下側
                cv.circle(image, landmark_point[477], 2, (0, 0, 255), -1)

        return image

    def draw_bounding_rect(self, use_brect, image, brect):
        if use_brect:
            # 外接矩形
            cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]), (0, 255, 0), 2)

        return image



    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = "camera"
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('rikibot_mediapipe', anonymous=True)
    facemesh = FaceMesh()
    rospy.spin()

if __name__ == '__main__':
    main()
