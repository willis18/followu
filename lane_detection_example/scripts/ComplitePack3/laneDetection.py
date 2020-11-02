#!/usr/bin/python3
######## LANE DETECTION STEER CONTROL ##########################################
#   IOELECTRON LANE CONTROL SYSTEM. by Parallel J.s
#   Ref Code By CAN OZCIVELEK
################################################################################


# IMPORT NECESSARY LIBRARIES
import cv2, os, time
import numpy as np
from matplotlib import pyplot as plt
import ioelectron_cardriver as ctrl_drv
import ioelectron_utils as utils

#region DATA INIT REGION -> Data Reference Control
cameraFeed= False
timeCycleChecker = False
#videoPath = "../drive_video/sample_drive.mp4"
#videoPath = "../drive_video/drive.avi"
#videoPath = "../drive_video/drive_dark.avi"
videoPath = "../drive_video/drive_high_angle.avi"
cameraNo= 0
cameraWidth = 1280
cameraHeight = 720
frameWidth = 640
frameHeight = 360
if cameraFeed:
    cap = cv2.VideoCapture(cameraNo)
    cap.set(3, cameraWidth)
    cap.set(4, cameraHeight)
else:
    cap = cv2.VideoCapture(videoPath)
intialTracbarVals = [10,20,0,100]
intialCarRef = 0
#endregion

#region timeCycle Checker
pasTime = time.time()
timeCycleBuf = ""
def timeCycle(point_name="INIT"):
    global timeCycleBuf
    global pasTime
    getTime = time.time()
    if point_name == "INIT":
        timeCycleBuf = "START(ms):[%0.1f]" % ((getTime - pasTime)*1000)
    elif point_name == "END":
        timeCycleBuf += "[%s %0.1f]" % (point_name, ((getTime - pasTime)*1000))
        print(timeCycleBuf)
    else:
        timeCycleBuf += "[%s %0.1f]" % (point_name, ((getTime - pasTime)*1000))
    pasTime = getTime
#endregion

#region SYSTEM START CODE BLOCK
success = False
success, wan_image = cap.read()
while not success:
    success, wan_image = cap.read()
    cv2.imshow(utils.trackbar_name, wan_image)

utils.initializeTrackbars(intialTracbarVals, intialCarRef)
utils.initializeTESTbar(intialTracbarVals)

curTime = time.time()
prevTime = curTime
src = utils.valTrackbars()
srcV = utils.readTestVal()
set_speed = utils.valTrackbar_speed()
mtx, miv = utils.perspectiveWarp_init(frameWidth, frameHeight, src)

pasTime = time.time()

dst_steer_level = 400
deviation = 0.0

processing_img = wan_image

while True:
    curTime = time.time()
    success, wan_image = cap.read()
    
    if not success: continue
    fps = 1 / (curTime - prevTime)
    prevTime = curTime
    fps_str = "FPS: %0.1f" % fps

    lineError = " OK"
    if timeCycleChecker: timeCycle()
    
    #region img_resize
    #CUDA CODE
    #wan_image_mat = cv2.cuda_GpuMat(wan_image)
    #resize_image_mat = cv2.cuda.resize(wan_image_mat, (frameWidth, frameHeight), interpolation = cv2.INTER_AREA)
    #image = resize_image_mat.download()
    #CPU CODE
    
    image = cv2.resize(wan_image, (frameWidth, frameHeight), interpolation = cv2.INTER_AREA)
    # Displaying final image
    #cv2.putText(image, fps_str, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
    #cv2.imshow("Final", image)
    # Wait for the Enter key to be pressed to stop
    #if cv2.waitKey(1) == 13: break
    #continue
    #endregion
    if timeCycleChecker: timeCycle("A")
    try:
        #region trackbar read
        #src = utils.valTrackbars()
        src = utils.valTrackbars()
        srcV = utils.readTestVal()
        if utils.trackbar_change_flag:
            utils.trackbar_change_flag = False
            print("Recalculate perspectiveWarp.")
            mtx, miv = utils.perspectiveWarp_init(frameWidth, frameHeight, src)
        
        if utils.speed_change_flag:
            utils.speed_change_flag = False
            set_speed = utils.valTrackbar_speed()
            print("Change the speed " + str(set_speed))
        #endregion
        if timeCycleChecker: timeCycle("B")

        #region image warp
        # Apply perspective warping by calling the "perspectiveWarp()" function
        # Then assign it to the variable called (birdView)
        # Provide this function with:
        # 1- an image to apply perspective warping (frame)
        #birdView, birdViewL, birdViewR = utils.perspectiveWarp(image, frameWidth, frameHeight, src, mtx)
        birdView = utils.perspectiveWarp(image, frameWidth, frameHeight, src, mtx)
        #endregion
        if timeCycleChecker: timeCycle("C")

        #region image process
        # Apply image processing by calling the "processImage()" function
        # Then assign their respective variables (img, hls, grayscale, thresh, blur, canny)
        # Provide this function with:
        # 1- an already perspective warped image to process (birdView)
        ##hls, grayscale, thresh, blur, canny = utils.processImage(birdView)
        processing_img = utils.processImage(birdView, srcV)
        #thresh = utils.processImage(birdView)
        #endregion
        if timeCycleChecker: timeCycle("D")

        #region histogram plot (plt.plot??)
        ##Not Using hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
        ##Not Using hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)

        # Plot and display the histogram by calling the "get_histogram()" function
        # Provide this function with:
        # 1- an image to calculate histogram on (thresh)
        #hist, leftBase, rightBase = utils.plotHistogram(thresh, frameWidth, frameHeight)
        hist = utils.plotHistogram(processing_img, frameWidth, frameHeight)
        # print(rightBase - leftBase)
        #plt.plot(hist)
        #plt.show()
        #endregion
        if timeCycleChecker: timeCycle("E")

        #region slide rectangle search (left and right multi core thread check...)
        ploty, left_fit, right_fit, left_fitx, right_fitx = utils.slide_window_search(processing_img, hist, frameWidth, frameHeight)
        #plt.plot(left_fit)
        #plt.show()
        #endregion
        if timeCycleChecker: timeCycle("F")

        #region general line search (left and right multi core thread check...)
        draw_info = utils.general_search(processing_img, left_fit, right_fit, frameWidth, frameHeight)
        # plt.show()
        #endregion
        if timeCycleChecker: timeCycle("G")

        #region measure lane (left and right multi core thread check...)
        curveRad, curveDir = utils.measure_lane_curvature(ploty, left_fitx, right_fitx)
        #endregion
        if timeCycleChecker: timeCycle("H")
        
        #region draw result lane
        # Filling the area of detected lanes with green
        meanPts, image = utils.draw_lane_lines(image, frameWidth, frameHeight, processing_img, miv, draw_info)
        #endregion
        if timeCycleChecker: timeCycle("I")

        #region offcenter check
        #deviation, directionDev = offCenter(meanPts, image, frameWidth, frameHeight)
        deviation, directionDev = utils.offCenter(meanPts, frameWidth)
        dst_steer_level = int(deviation*500)
        #endregion
        if timeCycleChecker: timeCycle("J")
    except Exception as e:
        print(str(e))
        lineError = " ERROR"

    #region Car Control defines fuction
    dst_steer_level = ctrl_drv.steer_center - dst_steer_level

    if dst_steer_level < ctrl_drv.steer_minimum: dst_steer_level = ctrl_drv.steer_minimum
    elif dst_steer_level > ctrl_drv.steer_maximum: dst_steer_level = ctrl_drv.steer_maximum

    auto_steer_control = 0
    car_vector = 0
    car_speed_level = set_speed * 20
    if ctrl_drv.ai_enabled != 0:
        auto_steer_control = 1
        if(car_speed_level > 0): car_vector = 1
    ctrl_drv.car_control_cmd(auto_steer_control, dst_steer_level, car_vector, car_speed_level, 0, 0, 0)
    #endregion
    if timeCycleChecker: timeCycle("K")

    #region Adding text to our final image
    # Adding text to our final image
    fps_str += " [BAT: %d]" % ctrl_drv.battery_level
    fps_str += "[EN%d]" % ctrl_drv.ai_enabled
    fps_str += "[CEN%d]" % ctrl_drv.steer_center
    fps_str += "[%d]" % dst_steer_level
    fps_str += lineError
    cv2.putText(image, fps_str, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
    #finalImg = utils.addText(result, curveRad, curveDir, deviation, directionDev, fps_str)
    #curveRad 커브 라디안 값
    #curveDir 커브될 방향
    #deviation 커브각도
    #directionDev
    #endregion
    if timeCycleChecker: timeCycle("L")
    
    # Displaying final image
    # Perspective points to be warped
    utils.drawPoints(image, frameWidth, frameHeight, src)
    cv2.imshow(utils.trackbar_name, image)
    cv2.imshow(utils.testbar_name, processing_img)

    if timeCycleChecker: timeCycle("END")
    # Wait for the Enter key to be pressed to stop
    if cv2.waitKey(1) == 13: break
#endregion

# Code End
ctrl_drv.thread_run = False
cv2.destroyAllWindows()