After catkin_make run :

rosrun final_trajectory sync -> it saves cordinates of person[in kinect's frame of reference] detected by tshirt color(by default green) into a txt file

rosrun final_trajectory sync_with_amcl -> additionally, it subscribes to amcl_pose and gets the pose and orientation of robot under robot_pose and robot_orientation.

-> Need to transform the cordinates accordingly to get the person's pose in world coordinates

How to change color of detection : final_trajectory sync additionally takes 6 more arguments[optional], these are :

final_trajectory sync lowercolor uppercolor lowersaturation uppersaturaion lowerlight upperlight

lowerlimit is 0 and upper is 255 for all 3 values.

First two parameters typically define the range for color eg, 30-90 for green ; 100-150 for blue.

Keep upper saturation and upperlight as 255.

Adjust lowersaturation and lowerlight according to illumination [In normal tubelight keep around 90 and 70 respectively]

For more details, Refer to : http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html

To see which color is being filtered its better to use this node :

rosrun trajectory_test traj ->shows the portion of image which is detected,usage same as above, takes additional 6 parameters

