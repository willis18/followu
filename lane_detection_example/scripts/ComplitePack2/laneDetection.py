#!/usr/bin/python3
# non optimize view all type

import numpy as np
import cv2, utlis, time 
import threading
import SerialToDriver as ctrl_drv

####################################################
cameraFeed= False
#videoPath = "../drive_video/sample_drive.mp4"
#videoPath = "../drive_video/drive.avi"
#videoPath = "../drive_video/drive_dark.avi"
videoPath = "../drive_video/drive_high_angle.avi"
cameraNo= 0
frameWidth= 640
frameHeight = 480

intialTracbarVals = [40, 30, 10, 80]
####################################################

if cameraFeed:
    cap = cv2.VideoCapture(cameraNo)
    cap.set(3, frameWidth)
    cap.set(4, frameHeight)
else:
    cap = cv2.VideoCapture(videoPath)

count=0
noOfArrayValues =10 
global arrayCurve, arrayCounter
arrayCounter=0
arrayCurve = np.zeros([noOfArrayValues])
myVals=[]
utlis.initializeTrackbars(intialTracbarVals)

curTime = time.time()
prevTime = curTime
averageCurve = 0.0

while(True):
    curTime = time.time()
    sec = curTime - prevTime
    fps = 1/(sec)
    
    #if fps > 25.0: continue
    prevTime = curTime
    fps_str = "FPS: %0.1f" % fps
    success, img = cap.read()

    #img = cv2.imread('test3.jpg')
    if cameraFeed== False:img = cv2.resize(img, (frameWidth, frameHeight), None)
    imgWarpPoints = img.copy()
    imgFinal = img.copy()
    imgCanny = img.copy()

    imgUndis = utlis.undistort(img)
    imgThres,imgCanny,imgColor = utlis.thresholding(imgUndis)
    src = utlis.valTrackbars()
    imgWarp = utlis.perspective_warp(imgThres, dst_size=(frameWidth, frameHeight), src=src)
    imgWarpPoints = utlis.drawPoints(imgWarpPoints, src)
    imgSliding, curves, lanes, ploty = utlis.sliding_window(imgWarp, nwindows=15, margin=50, draw_windows=True)

    try:
        lane_curve=0
        curverad =utlis.get_curve(imgFinal, curves[0], curves[1])
        lane_curve = np.mean([curverad[0], curverad[1]])
        imgFinal = utlis.draw_lanes(img, curves[0], curves[1],frameWidth,frameHeight,src=src)

        # ## Average
        currentCurve = lane_curve // 50
        if  int(np.sum(arrayCurve)) == 0: averageCurve = currentCurve
        else:
            averageCurve = np.sum(arrayCurve) // arrayCurve.shape[0]
        if abs(averageCurve-currentCurve) >200: arrayCurve[arrayCounter] = averageCurve
        else :arrayCurve[arrayCounter] = currentCurve
        arrayCounter +=1
        if arrayCounter >= noOfArrayValues : arrayCounter=0
        
        directionText = 'Non Range'
        if averageCurve > 10:
            directionText='Right'
        elif averageCurve < -10:
            directionText='Left'
        elif averageCurve <10 and averageCurve > -10:
            directionText='Straight'
        elif averageCurve == -1000000:
            directionText = 'No Lane Found'

        #print("avC:" + str(averageCurve) + "Dir:" + str(directionText))
        #cv2.putText(imgFinal, directionText, (frameWidth//2-70, 70), cv2.FONT_HERSHEY_DUPLEX, 1.75, (0, 0, 255), 2, cv2.LINE_AA)
    except:
        lane_curve=0
        pass

    imgFinal= utlis.drawLines(imgFinal,lane_curve)
    imgThres = cv2.cvtColor(imgThres,cv2.COLOR_GRAY2BGR)
    imgBlank = np.zeros_like(img)

    #region Car Control defines fuction
    dst_steer_level = ctrl_drv.steer_center + (averageCurve * 5)

    if dst_steer_level < ctrl_drv.steer_minimum: dst_steer_level = ctrl_drv.steer_minimum
    elif dst_steer_level > ctrl_drv.steer_maximum: dst_steer_level = ctrl_drv.steer_maximum

    auto_steer_control = 0
    car_vector = 0
    car_speed_level = 30 #target speed
    if ctrl_drv.ai_enabled != 0:
        auto_steer_control = 1
        if(car_speed_level > 0): car_vector = 1
    ctrl_drv.car_control_cmd(auto_steer_control, dst_steer_level, car_vector, car_speed_level, 0, 0, 0)
    #endregion

    fps_str += " [BAT: %d]" % ctrl_drv.battery_level
    fps_str += "[EN%d]" % ctrl_drv.ai_enabled
    fps_str += "[CEN%d]" % ctrl_drv.steer_center
    fps_str += "[%d]" % dst_steer_level

    cv2.putText(imgFinal, fps_str, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))

    imgStacked = utlis.stackImages(0.5, ([img,imgUndis,imgWarpPoints], [imgColor, imgCanny, imgThres], [imgWarp,imgSliding,imgFinal] ))
    #imgStacked1=utlis.stackImages(0.7, ([imgWarpPoints,imgWarp])
    cv2.imshow("PipeLine",imgStacked)
    #cv2.imshow("Result", imgFinal)

    if cv2.waitKey(1) == 13:
        break
        
#cap.release(int(averageCurve))
ctrl_drv.thread_run = False
cv2.destroyAllWindows() 
