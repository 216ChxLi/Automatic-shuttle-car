# Aufgabe 1: Tronis Situation

# Aufgabe 2: Lane Detection
1. Use Gaussian Blur with a 3*3 kernel to remove noise and smooth the image.
2. Convert the image from the colorful one to the gray one, which could be helpful for the following edge detection
3. Transform the image into binary matrics and use Canny function to detect the edge.
   Alternatively man can use Sobel Derivative here.  
   https://docs.opencv.org/3.4/d2/d2c/tutorial_sobel_derivatives.html
4. Creat a Polygon mask to avoid the influence from surroundings. And use bitwise_and() function to cover the mask on the image
   https://blog.csdn.net/u011028345/article/details/77278467
5. Use Hough Line Transform to get the lines of lanes. But remeber, on one lane could exist several lines. 
   https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
7. We need to fit the several lines from Hough Line Transform into one.
   a. Locate the line's position to make sure it is in the left or right part of the image
   b. Use all the points on the line to fit into one line.
   c. Set the start and end of the line. Show on the image

8. Additional Information:
1. For more explicit detection, man can transfer the image into bird eye's view
   https://nikolasent.github.io/opencv/2017/05/07/Bird%27s-Eye-View-Transformation.html
2. Simple Lane Detection with OpenCV
   https://medium.com/@mrhwick/simple-lane-detection-with-opencv-bfeb6ae54ec0
3. For detecting the yellow warning lines, we should add color distinguish function in the algorithms
   https://blog.csdn.net/qq7835144/article/details/95048117
   https://cloud.tencent.com/developer/article/1471688

# Aufgabe 3: Steering Control
1. Use the start and end point of the line to get the slope.
2. Set a PD controller to adjust the steering value.
3. Use the sign of slope to determine to turn left or right

# Aufgabe 4: Throttle Control
1. Set the velocity and Bounding Box Sensor to get the data from Tronis
2. Use the relative coordination to process the relative distance
3. Set a mindestens distance to make ego car stop and not crash to other cars on the road.
4. Set a PID controller to adjust the throttle value. When there is nothing on the road, ego car can run with a steady velocity.
