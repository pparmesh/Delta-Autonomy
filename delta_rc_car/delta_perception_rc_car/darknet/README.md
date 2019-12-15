# Rosbag files
- To record ros file - `rosrecord`
- To run robag file - `rosbag play -l yolo_train_2.bag`
- To view the rosbag file - `rosrun image_view image_view image:=/pt_grey/image_color_rect`
- To extract images from rosbag - follow - http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

# Label creation 
- yolo_mark
- Trained on 300 images
- 1 class
- YOLO V3

# Running trained model
- Download weights --> `https://drive.google.com/drive/u/4/folders/1S0hoJkS-99UUj2OeEakh3s62q82XAQXH`
- `./darknet detector test data/obj.data  cfg/yolo-rc_car.cfg backup/yolo-rc_car_2000.weights data_test/frame0473.jpg`
- (cfg data and names files are stored in yolo_train folder)
