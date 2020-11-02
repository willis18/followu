#!/usr/bin/python3

import cv2, os, threading
import numpy as np
from matplotlib import pyplot as plt
import pickle

ym_per_pix = 0.6 / 360   # 미터법 계산을 위한 픽셀값 조정값
xm_per_pix = 0.8 / 640  # 표준 차선 너비 / 가로폭 픽셀

# Get path to the current working directory
CWD_PATH = os.getcwd()

#region CODE BLOCK REGION -> IOELECTRON TRACKBAR CONTROL
trackbar_change_flag = False
speed_change_flag = False
def trackbar_ctrl(x):
    global trackbar_change_flag
    trackbar_change_flag = True
def speed_change(x):
    global speed_change_flag
    speed_change_flag = True

trackbar_name = "Control Panel"
#ioelectron_utils.initializeTrackbars(intialTracbarVals)
def initializeTrackbars(intialTracbarVals, intialCarRef):
    #intializing trackbars for region of intrest
    cv2.namedWindow(trackbar_name, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Top-X", trackbar_name, intialTracbarVals[0], 100, trackbar_ctrl)
    cv2.createTrackbar("Top-Y", trackbar_name, intialTracbarVals[1], 100, trackbar_ctrl)
    cv2.createTrackbar("Bottom-X", trackbar_name, intialTracbarVals[2], 100, trackbar_ctrl)
    cv2.createTrackbar("Bottom-Y", trackbar_name, intialTracbarVals[3], 100, trackbar_ctrl)
    cv2.createTrackbar("Speed", trackbar_name, intialCarRef, 95, speed_change)
    cv2.resizeWindow(trackbar_name, 400, 520)

#ioelectron_utils.valTrackbars()
def valTrackbars():
	#getting the values of ROI
    widthTop = cv2.getTrackbarPos("Top-X", trackbar_name)
    heightTop = cv2.getTrackbarPos("Top-Y", trackbar_name)
    widthBottom = cv2.getTrackbarPos("Bottom-X", trackbar_name)
    heightBottom = cv2.getTrackbarPos("Bottom-Y", trackbar_name)
    
    src = np.float32([(widthTop/100,heightTop/100), (1-(widthTop/100), heightTop/100),
                      (widthBottom/100, heightBottom/100), (1-(widthBottom/100), heightBottom/100)])
    #src = np.float32([widthTop, heightTop/100, widthBottom/100, heightBottom, speed_lv])
    #src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    return src
def valTrackbar_speed():
    speed_lv = cv2.getTrackbarPos("Speed", trackbar_name)
    return speed_lv
#endregion

#region CODE BLOCK REGION -> IOELECTRON TEST BAR CONTROL
testbar_name = "VariablePanel"

def TESTbar_nothing(x):
    pass

def initializeTESTbar(intialTestbarVals):
    #intializing trackbars for region of intrest
    cv2.namedWindow(testbar_name, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("REF1", testbar_name, intialTestbarVals[0], 255, TESTbar_nothing)
    cv2.createTrackbar("REF2", testbar_name, intialTestbarVals[1], 255, TESTbar_nothing)
    cv2.createTrackbar("REF3", testbar_name, intialTestbarVals[2], 255, TESTbar_nothing)
    cv2.createTrackbar("REF4", testbar_name, intialTestbarVals[3], 255, TESTbar_nothing)
    cv2.resizeWindow(testbar_name, 400, 520)

def readTestVal():
    src = []
    src.append(int(cv2.getTrackbarPos("REF1", testbar_name)))
    src.append(int(cv2.getTrackbarPos("REF2", testbar_name)))
    src.append(int(cv2.getTrackbarPos("REF3", testbar_name)))
    src.append(int(cv2.getTrackbarPos("REF4", testbar_name)))
    return src
    
#endregion

# Image Processing (CVT+HLS->CVT+GRAY->THRESHOLD->BLUR->CANNY) #################
def processImage(inpImage, srcV):
    # Apply HLS color filtering to filter out white lane lines
    #imgMat = cv2.UMat(inpImage)
    imgMat = inpImage
    hlsMat = cv2.cvtColor(imgMat, cv2.COLOR_BGR2HLS)
    lower_white = np.array([136, 164, 101]) #136 164 101
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(imgMat, lower_white, upper_white)
    imgMat = cv2.bitwise_and(imgMat, hlsMat, mask = mask)
    
    # Convert image to grayscale, apply threshold, blur & extract edges
    imgMat = cv2.cvtColor(imgMat, cv2.COLOR_BGR2GRAY)
    
    ret, imgMat = cv2.threshold(imgMat, 50, 255, cv2.THRESH_BINARY) #50 255
    
    imgMat = cv2.GaussianBlur(imgMat,(3, 3), 0)

    imgMat = cv2.Canny(imgMat, 40, 60) #40-60

    return imgMat
################################################################################

# Image Control mark point view img to include point marker ####################
def drawPoints(img, myWidth, myHeight, src):
	#drawing circles on frame
    img_size = np.float32([(myWidth, myHeight)])
    #src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    src = src * img_size
    for x in range( 0,4): 
        cv2.circle(img,(int(src[x][0]),int(src[x][1])),5,(0,255,0),cv2.FILLED)
    #return img
################################################################################

# Image Warp, insult mark point. (DRAW POINT->Perspective->Perspective->Perspective)
warp_arrays = np.float32([[0,0], [0,0], [0,0], [0,0]])
def perspectiveWarp_init(img_w, img_h, srcs):
    # Get image size
    img_size = (img_w, img_h)

    srcs *= img_size
    for x in range(0, 4):
        warp_arrays[x][0] = int(srcs[x][0])
        warp_arrays[x][1] = int(srcs[x][1])

    # Window to be shown
    dst = np.float32([[0, 0], [img_w, 0], [0, img_h], [img_w, img_h]])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(warp_arrays, dst)
    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, warp_arrays)

    return matrix, minv
def perspectiveWarp(inpImage, img_w, img_h, srcs, matrix_data):
    # Get image size
    img_size = (img_w, img_h)

    birdseye = cv2.warpPerspective(inpImage, matrix_data, img_size)

    # Divide the birdseye view into 2 halves to separate left & right lanes
    #birdseyeLeft  = birdseye[0:img_h, 0:img_w // 2]
    #birdseyeRight = birdseye[0:img_h, img_w // 2:img_w]

    # Display birdseye view image
    # cv2.imshow("Birdseye" , birdseye)
    # cv2.imshow("Birdseye Left" , birdseyeLeft)
    # cv2.imshow("Birdseye Right", birdseyeRight)

    return birdseye#, birdseyeLeft, birdseyeRight
################################################################################

# cal left, right lane for center histogram x point ############################
def plotHistogram(inpImage, img_w, img_h):

    histogram = np.sum(inpImage[img_h // 2:, :], axis = 0)

    ##midpoint = np.int(histogram.shape[0] / 2)
    #midpoint = np.int(img_w / 2)
    #leftxBase = np.argmax(histogram[:midpoint])
    #rightxBase = np.argmax(histogram[midpoint:]) + midpoint
    ##rightxBase = np.argmax(histogram[midpoint:])

    #plt.xlabel("Image X Coordinates")
    #plt.ylabel("Number of White Pixels")

    # Return histogram and x-coordinates of left & right lanes to calculate
    # lane width in pixels
    return histogram#, leftxBase, rightxBase
################################################################################

# Find the start of left and right lane ########################################
def slide_window_search(binary_warped, histogram, img_w, img_h):
    # Find the start of left and right lane lines using histogram info
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    #midpoint = np.int(histogram.shape[0] / 2)
    midpoint = np.int(img_w / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # A total of 9 windows will be used
    nwindows = 9
    #window_height = np.int(binary_warped.shape[0] / nwindows)
    window_height = np.int(img_h / nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100
    minpix = 80
    left_lane_inds = []
    right_lane_inds = []

    #### START - Loop to iterate through windows and search for lane lines #####
    for window in range(nwindows):
        #win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_low = img_h - (window + 1) * window_height
        #win_y_high = binary_warped.shape[0] - window * window_height
        win_y_high = img_h - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
        (0,255,0), 2)
        cv2.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
        (0,255,0), 2)
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
    #### END - Loop to iterate through windows and search for lane lines #######

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Apply 2nd degree polynomial fit to fit curves
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)


    #ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    ploty = np.linspace(0, img_h-1, img_h)
    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    #plt.plot(right_fitx)
    #plt.show()

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # plt.imshow(out_img)
    #plt.plot(left_fitx,  ploty, color = 'yellow')
    #plt.plot(right_fitx, ploty, color = 'yellow')
    #plt.xlim(0, img_w)
    #plt.ylim(img_h, 0)


    return ploty, left_fit, right_fit, ltx, rtx
################################################################################

# search lane.. ################################################################
def general_search(binary_warped, left_fit, right_fit, img_w, img_h):

    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 50
    
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) 
    & (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin)))

    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) 
    & (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    #ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    ploty = np.linspace(0, img_w-1, img_w)
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty

    return ret

    ## VISUALIZATION ###########################################################

    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    window_img = np.zeros_like(out_img)
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                  ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

    plt.imshow(result)
    plt.plot(left_fitx,  ploty, color = 'yellow')
    plt.plot(right_fitx, ploty, color = 'yellow')
    plt.xlim(0, img_w)
    plt.ylim(img_h, 0)

    return ret
################################################################################

# measure lane curve data ######################################################
def measure_lane_curvature(ploty, leftx, rightx):

    leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
    rightx = rightx[::-1]  # Reverse to match top-to-bottom in y

    # Choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)

    # Fit new polynomials to x, y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

    # Calculate the new radii of curvature
    left_curverad  = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
    # print(left_curverad, 'm', right_curverad, 'm')

    # Decide if it is a left or a right curve
    if leftx[0] - leftx[-1] > 60:
        curve_direction = 'Left Curve'
    elif leftx[-1] - leftx[0] > 60:
        curve_direction = 'Right Curve'
    else:
        curve_direction = 'Straight'

    return (left_curverad + right_curverad) / 2.0, curve_direction
################################################################################

# fill color to detected lane ##################################################
def draw_lane_lines(original_image, img_w, img_h, warped_image, Minv, draw_info):

    #leftx = draw_info['leftx']
    #rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    mean_x = np.mean((left_fitx, right_fitx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))

    #newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    newwarp = cv2.warpPerspective(color_warp, Minv, (img_w, img_h))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return pts_mean, result
################################################################################

# Calculating deviation in meters ##############################################
def offCenter(meanPts, inpFrame_w):
    # Calculating deviation in meters
    mpts = meanPts[-1][-1][-2].astype(int)
    #pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    pixelDeviation = inpFrame_w / 2 - abs(mpts)
    deviation = pixelDeviation * xm_per_pix
    direction = "left" if deviation < 0 else "right"

    return deviation, direction
################################################################################

# add text to result image #####################################################
def addText(img, radius, direction, deviation, devDirection, fps):

    # Add the radius and center position to the image
    font = cv2.FONT_HERSHEY_TRIPLEX

    if (direction != 'Straight'):
        text = 'Radius of Curvature: ' + '{:04.0f}'.format(radius) + 'm'
        text1 = 'Curve Direction: ' + (direction)

    else:
        text = 'Radius of Curvature: ' + 'N/A'
        text1 = 'Curve Direction: ' + (direction)

    cv2.putText(img, fps + " " + text , (50,100), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)
    cv2.putText(img, text1, (50,150), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)

    # Deviation
    deviation_text = 'Off Center: ' + str(round(abs(deviation), 3)) + 'm' + ' to the ' + devDirection
    cv2.putText(img, deviation_text, (50, 200), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0,100, 200), 2, cv2.LINE_AA)

    return img
################################################################################