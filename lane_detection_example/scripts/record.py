#!/usr/bin/python3

import datetime, cv2, time

cameraNo= 1
cameraWidth = 1280
cameraHeight = 720
cap = cv2.VideoCapture(cameraNo)
cap.set(3, cameraWidth)
cap.set(4, cameraHeight)
#cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H','2','6','4'))


now = datetime.datetime.now().strftime("%d_%H-%M-%S")
#fourcc = cv2.VideoWriter_fourcc(*'MP4V')
fourcc = cv2.VideoWriter_fourcc(*'XVID')
#video = cv2.VideoWriter(str(now) + ".MP4", fourcc, 20.0, (cameraWidth, cameraHeight))
video = cv2.VideoWriter(str(now) + ".avi", fourcc, 20.0, (cameraWidth, cameraHeight))

prevTime = curTime = time.time()
while True:
    success, wan_image = cap.read()
    #imgUMat = cv2.UMat(wan_image) #Python 3 version.
    imgUMat = wan_image
    
    if not success: continue
    video.write(imgUMat)

    imgUMat = cv2.resize(imgUMat, (int(cameraWidth/2), int(cameraHeight/2)), interpolation = cv2.INTER_AREA)
    cv2.imshow("VideoFrame", imgUMat)
    
    key = cv2.waitKey(1)
    if key == 13: break #ESC:27 ENTER:13

video.release()
cap.release()
cv2.destroyAllWindows()