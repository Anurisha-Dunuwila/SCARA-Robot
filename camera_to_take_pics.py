import cv2
import time 


#pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=15/1 ! videoconvert ! videoscale ! video/x-raw, width=640, height=480 ! appsink"

camera = cv2.VideoCapture(0)


time.sleep(2)

return_value, image = camera.read()

cv2.imshow("pre",image)
cv2.waitKey(5000)

cv2.imwrite('opencv.png', image)

camera.release()
cv2.destroyAllWindows()

#https://raspberrypi-guide.github.io/electronics/using-usb-webcams
