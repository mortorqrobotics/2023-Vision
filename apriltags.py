import cv2
import json

import pyapriltags as apriltag
import math
from scipy.spatial.transform import Rotation as R 
from pupil_apriltags import Detector
import numpy as np

with open('calibration_images/matrices.json', "r") as f:
   data = json.load(f)["camera_matrix"]["data"]

detector = Detector(
    families="tag16h5",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

logo = cv2.imread('image.png')
x = 100
y = 100
size = 100
logo = cv2.resize(logo, (size, size))

LINE_LENGTH = 5
CENTER_COLOR = (0, 255, 0)
CORNER_COLOR = (255, 0, 255)
width = 6 #in
px, dist1 = 380,16 # px_width of 191 at dist of 24 in
focal_length= (px*dist1)/width # for in
in_to_m = 0.0254
cam_prams = [data[0], data[4], data[2], data[5]]

def dist(pixel_width):
    return ((width * focal_length) / pixel_width)

### Some utility functions to simplify drawing on the camera feed
# draw a crosshair
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image,
                     (center[0] - LINE_LENGTH, center[1]),
                     (center[0] + LINE_LENGTH, center[1]),
                     color,
                     3)
    image = cv2.line(image,
                     (center[0], center[1] - LINE_LENGTH),
                     (center[0], center[1] + LINE_LENGTH),
                     color,
                     3)
    return image

# plot a little text
def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX,
                       1, color, 3)

# setup and the main loop
detector = apriltag.Detector()
cam = cv2.VideoCapture(0)

img2gray = cv2.cvtColor(logo, cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(img2gray, 1, 255, cv2.THRESH_BINARY)

while cam:
    result, image = cam.read()
    grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# look for tags
    detections = detector.detect(grayimg, estimate_tag_pose=True, tag_size=width*in_to_m, camera_params=cam_prams)
    if detections:
	    # found some tags, report them and update the camera image
        for detect in detections:
            image = plotPoint(image, detect.center, CENTER_COLOR)
            image = plotText(image, detect.center, CENTER_COLOR, detect.tag_id)
            for corner in detect.corners:
                image = plotPoint(image, corner, CORNER_COLOR)
            
            px_width = math.sqrt( ((detect.corners[1][0]-detect.corners[0][0])**2)+((detect.corners[1][0]-detect.corners[0][0])**2) )

            print(detect.corners[1][0])
            #print("ID: " + str(detect.tag_id))
            #print(px_width)
            print(focal_length)
            #print("Center: " + str(detect.center))
            print("Distance: " + str(dist(px_width)))
            #print("Rotation: " + str(R.from_matrix(detect.pose_R).as_euler('zxy', degrees=True)))
            #print(detect.pose_t)
            #print()


	# refresh the camera image
    cv2.imshow('Result', image)
	# let the system event loop do its thing
    key = cv2.waitKey(100)
	# terminate the loop if the 'Return' key his hit
    if key == 13:
        break

# loop over; clean up and dump the last updated frame for convenience of debugging
cv2.destroyAllWindows()
#cv2.imwrite("final.png", image)
cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)