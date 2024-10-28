# Copyright 2024 @with-RL
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

from youBot import YouBot
#import torch
#import cv2

#model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)


class KeyboardBot(YouBot):
    def run_step(self, count):
        # car control
        self.control_car()
        # arm control
        self.control_arm()
        # arm gripper
        self.control_gripper()
        # read lidarqa
        self.read_lidars()
        
        # read cameraa with YOLO
        img = self.read_camera_1()

        #results = model(img)

        #result_img = results.render()[0]

        #cv2.imshow("YOLO Detection", result_img)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #self.run_flag = False


if __name__ == "__main__":
    client = KeyboardBot()
    client.init_coppelia()
    client.run_coppelia()
