./darknet detector train cfg/custom.data cfg/yolov4-tiny-custom.cfg cfg/yolov4-tiny.conv.29 -gpus 0
./darknet detect cfg/yolov4-tiny-custom.cfg backup/yolov4-tiny-custom_last.weights 20210425-165453.jpg 
