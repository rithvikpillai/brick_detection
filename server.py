"""
Small server which wraps the Raspberry Pi camera device mounted on the end-effector, allowing multiple applications to use its data in parallel
"""
# Import Libraries
from __future__ import division
import argparse
import numpy as np
import cv2 as cv
import math
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import sys
from flask import Flask, send_file, jsonify

# Function 'setup_flash()' sets up the PWM instance for the GPIO pin 13 at a frequency of 1000 Hz and flashes the LED very quickly for 1 second *** RUN ONLY ONCE ***
def setup_flash():
    ledPin = 33 # MOSFET PWM is connected to GPIO 13 (Pin Header 33) on the Raspberry Pi
    global pwm
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(ledPin, GPIO.OUT)
    GPIO.output(ledPin, GPIO.LOW)
    pwm = GPIO.PWM(ledPin,1000)
    pwm.start(0)
    for i in range(10):
        time.sleep(0.05)
        pwm.ChangeDutyCycle(100)
        time.sleep(0.05)
        pwm.ChangeDutyCycle(0)



# 1 or 2 (int) if locally connected pi (Version x) camera. rtsp URI (string) if IP camera.
CAM = 2
#CAM = "rtsp://admin:@192.168.3.230"

RESOLUTION = 1        # resolution corresponding with id in `RESOLUTIONS`

# Dictionary of the available discrete input modes
RESOLUTIONS = {
    # V1 camera
    1: {
        1: (640 , 480),
        2: (1296, 730),
        3: (1296, 972),
        4: (2592, 1944)
    },
    # V2 camera
    2: {
        1: (1920, 1080), #	16:9	0.1-30fps	x	 	Partial	None
        2: (3280, 2464), #	 4:3	0.1-15fps	x	x	Full	None
        3: (3280, 2464), #	 4:3	0.1-15fps	x	x	Full	None
        4: (1640, 1232), #	 4:3	0.1-40fps	x	 	Full	2x2
        5: (1640,  922), #	16:9	0.1-40fps	x	 	Full	2x2
        6: (1280,  720), #	16:9	40-90fps	x	 	Partial	2x2
        7: (640 ,  480)  #	 4:3	40-90fps	x	 	Partial	2x2
    }
}

def capture():
    T_WAIT = 0.1           # time to wait (s) before frames are grabbed
    N_FRAMES = 10         # number of stills to grab

    CALIBRATION = True    # True iff images are meant for calibration. Alters filename.
    IS_PI_CAM = isinstance(CAM, int)

    try:
        # Use V4L2 if using the pi camera
        if IS_PI_CAM:
            cap = cv.VideoCapture(0, cv.CAP_V4L2)
        else:
            cap = cv.VideoCapture(CAM)

        # Check whether camera is opened successfully.
        if not (cap.isOpened()):
            print("Could not open video device")
        else:
            pwm.ChangeDutyCycle(100)
            for i in range(N_FRAMES):
                # Capture frame-by-frame
                ret, frame = cap.read()
                if ret:
                    if CALIBRATION:
                        fname = "capture/img_" + str(i) + ".png"
                    cv.imwrite(fname, frame)
    finally:
        pwm.ChangeDutyCycle(0)
        # When done, release the capture
        cap.release()# Take and save stills

def PasteandPrintLines(lines,image):
    line_image = np.copy(image) * 0
    for line in lines:
        line = line.astype(int).tolist()
        cv.line(line_image,(line[0],line[1]),(line[2],line[3]), (0, 255, 0), 1)
    img = cv.addWeighted(image, 0.8, line_image, 1, 0)
    #plt.figure(figsize=(10,10))
    #plt.imshow(img)
    
    return img
    
def computeAngle(x1,y1,x2,y2):
    if (x2-x1) != 0:
        slope = (y2-y1) / (x2-x1)
        angle = 180 * np.arctan(slope) / np.pi
    else:
        angle = 90
    return angle

def computeCenterDistance(x1,y1,x2,y2,x1_other,y1_other,x2_other,y2_other):
    x_mid = (x1 + x2)/2
    x_mid_other = (x1_other + x2_other)/2
    y_mid = (y1 + y2)/2
    y_mid_other = (y1_other + y2_other)/2
    center_dist = math.sqrt((x_mid - x_mid_other)**2 + (y_mid - y_mid_other)**2)
    
    return center_dist

def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def inter(L1,L2):                                                   
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return None
    
def intersection(LVline, THline, RVline, BHline):
    LV = line((LVline[0][0],LVline[0][1]),(LVline[0][2],LVline[0][3]))
    TH = line((THline[0][0],THline[0][1]),(THline[0][2],THline[0][3]))
    RV = line((RVline[0][0],RVline[0][1]),(RVline[0][2],RVline[0][3]))
    BH = line((BHline[0][0],BHline[0][1]),(BHline[0][2],BHline[0][3]))

    x1,y1 = inter(LV,TH)
    x2,y2 = inter(TH,RV)
    x3,y3 = inter(LV,BH)
    x4,y4 = inter(RV,BH)
    
    return x1,y1,x2,y2,x3,y3,x4,y4

def averageYaw(x1, y1, x2, y2, x3, y3, x4, y4):
    angles = [0] * 4
    angles[0] = computeAngle(x1,y1,x2,y2)
    angles[1] = computeAngle(x3,y3,x4,y4)
    angles[2] = computeAngle(x1,y1,x3,y3)
    angles[3] = computeAngle(x2,y2,x4,y4)

    if (angles[2]) < 0:
        angles[2] = angles[2] + 90
    else:
        if (angles[2]) > 0:
            angles[2] = 90 - angles[2]

    if (angles[3]) < 0:
        angles[3] = angles[3] + 90
    else:
        if (angles[3]) > 0:
            angles[3] = 90 - angles[3]
        
    yaw = sum(angles)/len(angles)

    return yaw

def centerOffset(x1, y1, x2, y2, x3, y3, x4, y4, scaling_factorX, scaling_factorY):
    x_mid1 = (x1 + x2) / 2
    x_mid2 = (x3 + x4) / 2

    x_mid = (x_mid1 + x_mid2) / 2

    y_mid1 = (y1 + y3) / 2
    y_mid2 = (y2 + y4) / 2

    y_mid = (y_mid1 + y_mid2) / 2

    midpoint = [int(x_mid), int(y_mid)]

    x_deviation = (320 - x_mid) * scaling_factorX
    y_deviation = (240 - y_mid) * scaling_factorY

    return x_deviation, y_deviation, x_mid, y_mid

def SplitIntoStraightLines(lines, threshold):
    n = len(lines)
    angles = [None] * n
    lines_straight = []
    Vlines = []
    Hlines = []

    # Computation of Line Angles
    for idx,line in enumerate(lines):
        for x1,y1,x2,y2 in line:
            if (x2-x1) != 0:
                slope = (y2 - y1) / (x2 - x1)
                angles[idx] = 180 * np.arctan(slope) / np.pi
            else:
                angles[idx] = 90

    # Check if angles are close to -90, 90 or 0 and sort them accordingly
        if (((angles[idx] >= (-90 - threshold)) and (angles[idx] <= (-90 + threshold))) or ((angles[idx] >= (90 - threshold)) and (angles[idx] <= (90 + threshold)))):
            lines_straight.append(line)
            Vlines.append(line)
        if ((angles[idx] >= (0 - threshold)) and (angles[idx] <= (0 + threshold))):
            lines_straight.append(line)
            Hlines.append(line)
            
    return Vlines, Hlines

def ExtendLines(Vlines, Hlines):
    # Extend lines through the entire image
    Vlines_ext = []
    Hlines_ext = []

    for line in Vlines:
        for x1,y1,x2,y2 in line:
            y1_ext = 0
            y2_ext = 480

            if (x2-x1) != 0:
                m = (y2 - y1) / float(x2 - x1)
                b = y2 - m * x2
                x1_ext = (y1_ext - b) / m
                x2_ext = (y2_ext - b) / m
            else:
                x1_ext = x1
                x2_ext = x2

            Vlines_ext.append(np.array([x1_ext, y1_ext, x2_ext, y2_ext]))

    for line in Hlines:
        for x1,y1,x2,y2 in line:
            x1_ext = 0
            x2_ext = 640

            if (y2-y1) != 0:
                m = float(y2 - y1) / float(x2 - x1)
                b = y2 - m * x2
                y1_ext = m*x1_ext + b
                y2_ext = m*x2_ext + b
            else:
                y1_ext = y1
                y2_ext = y2

            Hlines_ext.append(np.array([x1_ext, y1_ext, x2_ext, y2_ext]))
    
    return Vlines_ext, Hlines_ext

def SplitIntoLeftRightTopBottom(Vlines_ext, Hlines_ext):
    LVlines = []
    RVlines = []
    THlines = []
    BHlines = []

    for line in Vlines_ext:
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        if (x1 < 320 and x2 < 320):
            LVlines.append(line)
        if (x2 > 320 and x2 > 320):
            RVlines.append(line)

    for line in Hlines_ext:
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        if (y1 < 240 and y2 < 240):
            THlines.append(line)
        if (y2 > 240 and y2 > 240):
            BHlines.append(line)
            
    return LVlines, RVlines, THlines, BHlines

def scoreAspect(lines1, lines2, threshold, weight, linetype, bricktype):
    n = len(lines1)
    m = len(lines2)
    scores = [1] * n
    scores_other = [1] * m
    
    if bricktype == "full":
        brick_length = 490
        brick_width = 233
    if bricktype == "half":
        brick_length = 245
        brick_width = 230
        
    if linetype == "vertical":
        brick_dimension = brick_length
    if linetype == "horizontal":
        brick_dimension = brick_width
        
    for idx, line in enumerate(lines1):
        
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        for other, other_line in enumerate(lines2):
            
            x1_other = other_line[0]
            y1_other = other_line[1]
            x2_other = other_line[2]
            y2_other = other_line[3]
            
#             x_mid1 = (x1 + x2)/4
#             x_mid_other1 = (x1_other + x2_other)/4
#             y_mid1 = (y1 + y2)/4
#             y_mid_other1 = (y1_other + y2_other)/4
#             center_dist1 = math.sqrt((x_mid1 - x_mid_other1)**2 + (y_mid1 - y_mid_other1)**2)
    
            x_mid2 = (x1 + x2)/2
            x_mid_other2 = (x1_other + x2_other)/2
            y_mid2 = (y1 + y2)/2
            y_mid_other2 = (y1_other + y2_other)/2
            center_dist2 = math.sqrt((x_mid2 - x_mid_other2)**2 + (y_mid2 - y_mid_other2)**2)

#             x_mid3 = (x1 + x2)* (3/4)
#             x_mid_other3 = (x1_other + x2_other) * (3/4)
#             y_mid3 = (y1 + y2) * (3/4)
#             y_mid_other3 = (y1_other + y2_other) * (3/4)
#             center_dist3 = math.sqrt((x_mid3 - x_mid_other3)**2 + (y_mid3 - y_mid_other3)**2)
            
            
#             if (brick_dimension - threshold) <= center_dist1 <= (brick_dimension + threshold):
#                     scores[idx] += 1
#                     scores_other[other] += 1
            if (brick_dimension - threshold) <= center_dist2 <= (brick_dimension + threshold):
                    scores[idx] += 1 * (scores_other[other] / max(scores_other))
                    scores_other[other] += 1 * (scores[idx] / max(scores))
#             if (brick_dimension - threshold) <= center_dist3 <= (brick_dimension + threshold):
#                     scores[idx] += 1
#                     scores_other[other] += 1
                    
    for score in scores:
        score = (score / max(scores)) * weight 
    for other in scores_other:
        other = (other / max(scores_other)) * weight                            
    return scores,scores_other

def scoreParallel(lines1, lines2, threshold, weight):
    n = len(lines1)
    m = len(lines2)
    scores = [1] * n
    scores_other = [1] * m
    
        
    for idx, line in enumerate(lines1):
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        angle = computeAngle(x1, y1, x2, y2)
        
        for other, other_line in enumerate(lines2):
            x1_other = other_line[0]
            y1_other = other_line[1]
            x2_other = other_line[2]
            y2_other = other_line[3]
            angle_other = computeAngle(x1_other, y1_other, x2_other, y2_other)

            if (angle_other - threshold) <= angle <= (angle_other + threshold):
                scores[idx] += 1
                scores_other[other] += 1
    
    for score in scores:
        score = (score / max(scores)) * weight
    for other in scores_other:
        other = (other / max(scores_other)) * weight
    return scores, scores_other

def scorePerpendicular(lines1, lines2, threshold, weight):
    n = len(lines1)
    m = len(lines2)
    scores = [1] * n
    scores_other = [1] * m

    for idx, line in enumerate(lines1):
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        angle = computeAngle(x1, y1, x2, y2)
        
        for j, perp_line in enumerate(lines2):
            x1_perp = perp_line[0]
            y1_perp = perp_line[1]
            x2_perp = perp_line[2]
            y2_perp = perp_line[3]
            perp_angle = computeAngle(x1_perp, y1_perp, x2_perp, y2_perp)
            
            if ((perp_angle - threshold) <= angle - 90  <= (perp_angle + threshold)) or ((perp_angle - threshold) <= angle + 90  <= (perp_angle + threshold)):
                scores[idx] += 1
                scores_other[j] += 1
                
    for score in scores:
        score = (score / max(scores)) * weight 
    for other in scores_other:
        other = (other / max(scores_other)) * weight 
    return scores, scores_other

def scoreGradient(Vlines, Hlines, gradient, step, weight):
    n = len(Vlines)
    scoresV = [0] * n
    for idx, line in enumerate(Vlines):
        
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        if (x2-x1) != 0:
            m = (y2 - y1) / float(x2 - x1)
            b = y2 - m * x2
            if x1 < x2:
                lowerbound = int(x1)
                upperbound = int(x2)
            else:
                lowerbound = int(x2)
                upperbound = int(x1)
            for x in range(lowerbound,upperbound):
                y = m*x + b
                if y >= 480:
                    y = 479
                if y < 0:
                    y = 0
                    
                if x >= 640:
                    x = 639
                if x < 0:
                    x = 0
            
                scoresV[idx] += gradient[int(y)][int(x)]
                x = x + step
        else:
            if y1 < y2:
                lowerbound = int(y1)
                upperbound = int(y2)
            else:
                lowerbound = int(y2)
                upperbound = int(y1)
            for y in range(int(y1),int(y2)):
                x = x1                   
                if y >= 480:
                    y = 479
                if y < 0:
                    y = 0
                    
                if x >= 640:
                    x = 639
                if x < 0:
                    x = 0
                scoresV[idx] += gradient[int(y)][int(x)]
                y = y + step

    m = len(Hlines)
    scoresH = [0] * m
    for idx, line in enumerate(Hlines):
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        if (y2-y1) != 0:
            m = float(y2 - y1) / float(x2 - x1)
            b = y2 - m * x2
            if y1 < y2:
                lowerbound = int(y1)
                upperbound = int(y2)
            else:
                lowerbound = int(y2)
                upperbound = int(y1)
            for y in range(lowerbound,upperbound):
                x = (y - b) / m
                if y >= 480:
                    y = 479
                if y < 0:
                    y = 0
                    
                if x >= 640:
                    x = 639
                if x < 0:
                    x = 0
            
                scoresH[idx] += gradient[int(y)][int(x)]
                y = y + step
        else:
            if x1 < x2:
                lowerbound = int(x1)
                upperbound = int(x2)
            else:
                lowerbound = int(x2)
                upperbound = int(x1)
            for y in range(int(x1),int(x2)):
                y = y1
                
                if y >= 480:
                    y = 479
                if y < 0:
                    y = 0
                    
                if x >= 640:
                    x = 639
                if x < 0:
                    x = 0
                    
                scoresH[idx] += gradient[int(y)][int(x)]
                x = x + step
            
    scoresV = (scoresV / max(scoresV)) * weight
    scoresH = (scoresH / max(scoresH)) * weight
    return scoresV, scoresH

def ChooseBestLines(scores,lines, outputArraySize):
    bestLines = []
    sortedScores = np.argsort(scores)[-outputArraySize:]

    for idx in sortedScores:
        if (scores[idx] != 0):
            bestLines.append(lines[idx])
    return bestLines

def process_image(fname):
    img_rgb = cv.imread(fname)
    # img_rgb = cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB)

    # Apply Clahe to RGB image (RGB -> LAB -> Split -> Apply to L -> Recombine -> Contrast Improved RGB)
    img_lab = cv.cvtColor(img_rgb, cv.COLOR_RGB2LAB)
    lab_planes = list(cv.split(img_lab))
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(5,5))
    lab_planes[0] = clahe.apply(lab_planes[0])
    img_lab = cv.merge(lab_planes)
    img_rgb_clahe = cv.cvtColor(img_lab, cv.COLOR_LAB2RGB)

    # Blur Image to Reduce Texturing
    img_bilateral = cv.bilateralFilter(img_rgb_clahe, 15, 75, 75)
    img_blur = cv.GaussianBlur(img_bilateral, (3,3),0)

    # Canny Edge Detection
    edges = cv.Canny(image=img_blur, threshold1=150, threshold2=200)

    # Sobel Gradient Detection
    scale = 1
    delta = 0
    ddepth = cv.CV_16S

    gray = cv.cvtColor(img_blur, cv.COLOR_RGB2GRAY)
    grad_x = cv.Sobel(gray, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
    grad_y = cv.Sobel(gray, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
    abs_grad_x = cv.convertScaleAbs(grad_x)
    abs_grad_y = cv.convertScaleAbs(grad_y)
    grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)

    # Hough Line Probability Determination
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 50  # minimum number of pixels making up a line
    max_line_gap = 30  # maximum gap in pixels between connectable line segments
    line_image = np.copy(img_rgb) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                        min_line_length, max_line_gap)

    return(img_rgb, lines, grad)


app = Flask("effectorcamera_server")

@app.route("/process", methods=["GET"])
def process():
    yawAll = []
    x_offsetAll = []
    y_offsetAll = []
    euclidAll = []
    lines4 = []
    x_midAll = []
    y_midAll = []
    scalingfactorX = 210/490
    scalingfactorY = 100/233
    capture()
    for i in range(0,9):
        fname = "capture/img_" + str(i) + ".png"
        # Read original image and convert to appropriate color profiles
        img_rgb, lines, grad = process_image(fname)
        # Remove Noisy Random Lines
        Vlines, Hlines = SplitIntoStraightLines(lines, threshold=15)

        # Extend Straight Lines Across Entire Frame
        Vlines_ext, Hlines_ext = ExtendLines(Vlines, Hlines)
        lines_ext = Hlines_ext + Vlines_ext
        #img_ext = PasteandPrintLines(lines_ext, img_rgb)

        # Split lines into Left, Right, Top, Bottom Categories
        LVlines, RVlines, THlines, BHlines = SplitIntoLeftRightTopBottom(Vlines_ext, Hlines_ext)

        # First Scoring Structure based on Aspect Ratio to remove noise
        LVscores00, RVscores00 = scoreAspect(lines1=LVlines, lines2=RVlines, threshold=5, weight=1, linetype="vertical", bricktype="full")
        THscores00, BHscores00 = scoreAspect(lines1=THlines, lines2=BHlines, threshold=5, weight=1, linetype="horizontal", bricktype="full")
        LVscores01, RVscores01 = scoreParallel(lines1=LVlines, lines2=RVlines, threshold=2, weight=0.5)
        THscores01, BHscores01 = scoreParallel(lines1=THlines, lines2=BHlines, threshold=2, weight=0.5)
        LVscores = [sum(x) for x in zip(LVscores00, LVscores01)]
        RVscores = [sum(x) for x in zip(RVscores00, RVscores01)]
        THscores = [sum(x) for x in zip(THscores00, THscores01)]
        BHscores = [sum(x) for x in zip(BHscores00, BHscores01)]
        LVlines1 = ChooseBestLines(LVscores,LVlines, 5)
        RVlines1 = ChooseBestLines(RVscores,RVlines, 5)
        THlines1 = ChooseBestLines(THscores,THlines, 5)
        BHlines1 = ChooseBestLines(BHscores,BHlines, 5)
        lines1 = LVlines1 + RVlines1 + THlines1 + BHlines1
        #img1 = PasteandPrintLines(lines1, img_rgb)

        # Second Scoring Structure based on Aspect, Parallel, Perpendicular and Gradient
        LVscores10, RVscores10 = scoreAspect(lines1=LVlines1, lines2=RVlines1, threshold=5, weight=1, linetype="vertical", bricktype="full")
        THscores10, BHscores10 = scoreAspect(lines1=THlines1, lines2=BHlines1, threshold=5, weight=1, linetype="horizontal", bricktype="full")
        LVscores11, RVscores11 = scoreParallel(lines1=LVlines1, lines2=RVlines1, threshold=2, weight=0.7)
        THscores11, BHscores11 = scoreParallel(lines1=THlines1, lines2=BHlines1, threshold=2, weight=0.7)
        LVscores12, THscores12 = scorePerpendicular(lines1=LVlines1, lines2=THlines1, threshold=2, weight=0.7)
        LVscores12, BHscores12 = scorePerpendicular(lines1=LVlines1, lines2=BHlines1, threshold=2, weight=0.7)
        RVscores12, THscores12 = scorePerpendicular(lines1=RVlines1, lines2=THlines1, threshold=2, weight=0.7)
        RVscores12, BHscores12 = scorePerpendicular(lines1=RVlines1, lines2=BHlines1, threshold=2, weight=0.7)
        LVscores13, THscores13 = scoreGradient(Vlines=LVlines1, Hlines=THlines1, gradient=grad, step=1, weight=0.2)
        RVscores13, BHscores13 = scoreGradient(Vlines=RVlines1, Hlines=BHlines1, gradient=grad, step=1, weight=0.2)
        LVscores1 = [sum(x) for x in zip(LVscores10, LVscores11, LVscores12, LVscores13)]
        RVscores1 = [sum(x) for x in zip(RVscores10, RVscores11, RVscores12, RVscores13)]
        THscores1 = [sum(x) for x in zip(THscores10, THscores11, THscores12, THscores13)]
        BHscores1 = [sum(x) for x in zip(BHscores10, BHscores11, BHscores12, BHscores13)]
        LVlines2 = ChooseBestLines(LVscores1,LVlines1, 3)
        RVlines2 = ChooseBestLines(RVscores1,RVlines1, 3)
        THlines2 = ChooseBestLines(THscores1,THlines1, 3)
        BHlines2 = ChooseBestLines(BHscores1,BHlines1, 3)
        lines2 = LVlines2 + RVlines2 + THlines2 + BHlines2
        #img2 = PasteandPrintLines(lines2, img_rgb)

        # Third Scoring Structure based on Aspect, Parallel, and Perpendicular
        LVscores20, RVscores20 = scoreAspect(lines1=LVlines2, lines2=RVlines2, threshold=1, weight=1, linetype="vertical", bricktype="full")
        THscores20, BHscores20 = scoreAspect(lines1=THlines2, lines2=BHlines2, threshold=1, weight=1, linetype="horizontal", bricktype="full")
        LVscores21, RVscores21 = scoreParallel(lines1=LVlines2, lines2=RVlines2, threshold=2, weight=0.5)
        THscores21, BHscores21 = scoreParallel(lines1=THlines2, lines2=BHlines2, threshold=2, weight=0.5)
        LVscores22, THscores22 = scorePerpendicular(lines1=LVlines2, lines2=THlines2, threshold=2, weight=0.5)
        LVscores22, BHscores22 = scorePerpendicular(lines1=LVlines2, lines2=BHlines2, threshold=2, weight=0.5)
        RVscores22, THscores22 = scorePerpendicular(lines1=RVlines2, lines2=THlines2, threshold=2, weight=0.5)
        RVscores22, BHscores22 = scorePerpendicular(lines1=RVlines2, lines2=BHlines2, threshold=2, weight=0.5)
        LVscores23, THscores23 = scoreGradient(Vlines=LVlines2, Hlines=THlines2, gradient=grad, step=1, weight=0.5)
        RVscores23, BHscores23 = scoreGradient(Vlines=RVlines2, Hlines=BHlines2, gradient=grad, step=1, weight=0.5)
        LVscores2 = [sum(x) for x in zip(LVscores20, LVscores21, LVscores22, LVscores23)]
        RVscores2 = [sum(x) for x in zip(RVscores20, RVscores21, RVscores22, RVscores23)]
        THscores2 = [sum(x) for x in zip(THscores20, THscores21, THscores22, THscores23)]
        BHscores2 = [sum(x) for x in zip(BHscores20, BHscores21, BHscores22, BHscores23)]
        LVline = ChooseBestLines(LVscores2,LVlines2, 1)
        RVline = ChooseBestLines(RVscores2,RVlines2, 1)
        THline = ChooseBestLines(THscores2,THlines2, 1)
        BHline = ChooseBestLines(BHscores2,BHlines2, 1)
        lines3 = LVline + RVline + THline + BHline
        # img3 = PasteandPrintLines(lines3, img_rgb)
        #
        # images = [img_ext, img1, img2, img3]
        #
        # plt.figure(figsize=(20, 20))
        # for i in range(4):
        #     plt.subplot(1,4,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)

        Vcenterdist = computeCenterDistance(LVline[0][0],LVline[0][1],LVline[0][2],LVline[0][3],RVline[0][0],RVline[0][1],RVline[0][2],RVline[0][3])
        Hcenterdist = computeCenterDistance(THline[0][0],THline[0][1],THline[0][2],THline[0][3],BHline[0][0],BHline[0][1],BHline[0][2],BHline[0][3])
        if (230 <= Hcenterdist <= 240) and (480 <= Vcenterdist <= 500):
            # Determination of Selected Line Intersections, Yaw, and X-Y offsets
            x1,y1,x2,y2,x3,y3,x4,y4 = intersection(LVline, THline, RVline, BHline)
            yaw = averageYaw(x1, y1, x2, y2, x3, y3, x4, y4)

            x_deviation, y_deviation, x_mid, y_mid = centerOffset(x1, y1, x2, y2, x3, y3, x4, y4, scalingfactorX, scalingfactorY)
            euclid = math.sqrt(x_deviation**2 + y_deviation ** 2)

            yawAll.append(yaw)
            x_midAll.append(x_mid)
            y_midAll.append(y_mid)
            x_offsetAll.append(x_deviation)
            y_offsetAll.append(y_deviation)
            euclidAll.append(euclid)
            lines4.append(LVline + RVline + THline + BHline)


    if (len(euclidAll) % 2 == 0):
        #print("even!")
        sortedEuclid = euclidAll
        sortedEuclid.sort()
        m = int(len(sortedEuclid) / 2) - 1
        m_next = m + 1
        median1 = sortedEuclid[m]
        median2 = sortedEuclid[m_next]
        index = euclidAll.index(median1)
        index_next = euclidAll.index(median2)

        finalyaw = (yawAll[index] + yawAll[index_next]) / 2
        finalX = (x_offsetAll[index] + x_offsetAll[index_next]) / 2
        finalY = (y_offsetAll[index] + y_offsetAll[index_next]) / 2
        finalX_mid = (x_midAll[index] + x_midAll[index_next]) / 2
        finalY_mid = (y_midAll[index] + y_midAll[index_next]) / 2
        # print("Run:",j)
        # print("Yaw:", finalyaw)
        # print("X:", finalX)
        # print("Y:", finalY)
        # print("\n")
        # img4 = PasteandPrintLines(lines4[index], img_rgb)
        # img5 = PasteandPrintLines(lines4[index_next], img_rgb)
        # img6 = cv.circle(img4, (int(finalX_mid),int(finalY_mid)), radius=2, color=(0, 255, 0), thickness=-1)
        # img7 = cv.circle(img6, (320,240), radius=2, color=(0, 0, 255), thickness=-1)
        #
        # images = [img4, img7]
        # plt.figure(figsize=(30, 30))
        # for i in range(2):
        #     plt.subplot(1,2,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)

    else:
        #print("odd!")
        euclidMedian = np.median(euclidAll)
        index = euclidAll.index(euclidMedian)
        finalyaw = yawAll[index]
        finalX = x_offsetAll[index]
        finalY = y_offsetAll[index]
        # print("Run:",j)
        # print("Yaw:", finalyaw)
        # print("X:", finalX)
        # print("Y:", finalY)
        # print("\n")
        # img4 = PasteandPrintLines(lines4[index], img_rgb)
        # img5 = cv.circle(img4, (int(x_midAll[index]),int(y_midAll[index])), radius=2, color=(0, 255, 0), thickness=-1)
        # img6 = cv.circle(img5, (320,240), radius=2, color=(0, 0, 255), thickness=-1)
        #
        # images = [img4, img6]
        # plt.figure(figsize=(20, 20))
        # for i in range(2):
        #     plt.subplot(1,2,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)
    
    return jsonify(yaw = finalyaw, x = finalX, y = finalY)


@app.route("/debug", methods=["GET"])
def debug():
    yawAll = []
    x_offsetAll = []
    y_offsetAll = []
    euclidAll = []
    lines4 = []
    x_midAll = []
    y_midAll = []
    scalingfactorX = 210/490
    scalingfactorY = 100/233
    capture()
    for i in range(0,9):
        fname = "capture/img_" + str(i) + ".png"
        # Read original image and convert to appropriate color profiles
        img_rgb, lines, grad = process_image(fname)
        # Remove Noisy Random Lines
        Vlines, Hlines = SplitIntoStraightLines(lines, threshold=15)

        # Extend Straight Lines Across Entire Frame
        Vlines_ext, Hlines_ext = ExtendLines(Vlines, Hlines)
        lines_ext = Hlines_ext + Vlines_ext
        #img_ext = PasteandPrintLines(lines_ext, img_rgb)

        # Split lines into Left, Right, Top, Bottom Categories
        LVlines, RVlines, THlines, BHlines = SplitIntoLeftRightTopBottom(Vlines_ext, Hlines_ext)

        # First Scoring Structure based on Aspect Ratio to remove noise
        LVscores00, RVscores00 = scoreAspect(lines1=LVlines, lines2=RVlines, threshold=5, weight=1, linetype="vertical", bricktype="full")
        THscores00, BHscores00 = scoreAspect(lines1=THlines, lines2=BHlines, threshold=5, weight=1, linetype="horizontal", bricktype="full")
        LVscores01, RVscores01 = scoreParallel(lines1=LVlines, lines2=RVlines, threshold=2, weight=0.5)
        THscores01, BHscores01 = scoreParallel(lines1=THlines, lines2=BHlines, threshold=2, weight=0.5)
        LVscores = [sum(x) for x in zip(LVscores00, LVscores01)]
        RVscores = [sum(x) for x in zip(RVscores00, RVscores01)]
        THscores = [sum(x) for x in zip(THscores00, THscores01)]
        BHscores = [sum(x) for x in zip(BHscores00, BHscores01)]
        LVlines1 = ChooseBestLines(LVscores,LVlines, 5)
        RVlines1 = ChooseBestLines(RVscores,RVlines, 5)
        THlines1 = ChooseBestLines(THscores,THlines, 5)
        BHlines1 = ChooseBestLines(BHscores,BHlines, 5)
        lines1 = LVlines1 + RVlines1 + THlines1 + BHlines1
        #img1 = PasteandPrintLines(lines1, img_rgb)

        # Second Scoring Structure based on Aspect, Parallel, Perpendicular and Gradient
        LVscores10, RVscores10 = scoreAspect(lines1=LVlines1, lines2=RVlines1, threshold=5, weight=1, linetype="vertical", bricktype="full")
        THscores10, BHscores10 = scoreAspect(lines1=THlines1, lines2=BHlines1, threshold=5, weight=1, linetype="horizontal", bricktype="full")
        LVscores11, RVscores11 = scoreParallel(lines1=LVlines1, lines2=RVlines1, threshold=2, weight=0.7)
        THscores11, BHscores11 = scoreParallel(lines1=THlines1, lines2=BHlines1, threshold=2, weight=0.7)
        LVscores12, THscores12 = scorePerpendicular(lines1=LVlines1, lines2=THlines1, threshold=2, weight=0.7)
        LVscores12, BHscores12 = scorePerpendicular(lines1=LVlines1, lines2=BHlines1, threshold=2, weight=0.7)
        RVscores12, THscores12 = scorePerpendicular(lines1=RVlines1, lines2=THlines1, threshold=2, weight=0.7)
        RVscores12, BHscores12 = scorePerpendicular(lines1=RVlines1, lines2=BHlines1, threshold=2, weight=0.7)
        LVscores13, THscores13 = scoreGradient(Vlines=LVlines1, Hlines=THlines1, gradient=grad, step=1, weight=0.2)
        RVscores13, BHscores13 = scoreGradient(Vlines=RVlines1, Hlines=BHlines1, gradient=grad, step=1, weight=0.2)
        LVscores1 = [sum(x) for x in zip(LVscores10, LVscores11, LVscores12, LVscores13)]
        RVscores1 = [sum(x) for x in zip(RVscores10, RVscores11, RVscores12, RVscores13)]
        THscores1 = [sum(x) for x in zip(THscores10, THscores11, THscores12, THscores13)]
        BHscores1 = [sum(x) for x in zip(BHscores10, BHscores11, BHscores12, BHscores13)]
        LVlines2 = ChooseBestLines(LVscores1,LVlines1, 3)
        RVlines2 = ChooseBestLines(RVscores1,RVlines1, 3)
        THlines2 = ChooseBestLines(THscores1,THlines1, 3)
        BHlines2 = ChooseBestLines(BHscores1,BHlines1, 3)
        lines2 = LVlines2 + RVlines2 + THlines2 + BHlines2
        #img2 = PasteandPrintLines(lines2, img_rgb)

        # Third Scoring Structure based on Aspect, Parallel, and Perpendicular
        LVscores20, RVscores20 = scoreAspect(lines1=LVlines2, lines2=RVlines2, threshold=1, weight=1, linetype="vertical", bricktype="full")
        THscores20, BHscores20 = scoreAspect(lines1=THlines2, lines2=BHlines2, threshold=1, weight=1, linetype="horizontal", bricktype="full")
        LVscores21, RVscores21 = scoreParallel(lines1=LVlines2, lines2=RVlines2, threshold=2, weight=0.5)
        THscores21, BHscores21 = scoreParallel(lines1=THlines2, lines2=BHlines2, threshold=2, weight=0.5)
        LVscores22, THscores22 = scorePerpendicular(lines1=LVlines2, lines2=THlines2, threshold=2, weight=0.5)
        LVscores22, BHscores22 = scorePerpendicular(lines1=LVlines2, lines2=BHlines2, threshold=2, weight=0.5)
        RVscores22, THscores22 = scorePerpendicular(lines1=RVlines2, lines2=THlines2, threshold=2, weight=0.5)
        RVscores22, BHscores22 = scorePerpendicular(lines1=RVlines2, lines2=BHlines2, threshold=2, weight=0.5)
        LVscores23, THscores23 = scoreGradient(Vlines=LVlines2, Hlines=THlines2, gradient=grad, step=1, weight=0.5)
        RVscores23, BHscores23 = scoreGradient(Vlines=RVlines2, Hlines=BHlines2, gradient=grad, step=1, weight=0.5)
        LVscores2 = [sum(x) for x in zip(LVscores20, LVscores21, LVscores22, LVscores23)]
        RVscores2 = [sum(x) for x in zip(RVscores20, RVscores21, RVscores22, RVscores23)]
        THscores2 = [sum(x) for x in zip(THscores20, THscores21, THscores22, THscores23)]
        BHscores2 = [sum(x) for x in zip(BHscores20, BHscores21, BHscores22, BHscores23)]
        LVline = ChooseBestLines(LVscores2,LVlines2, 1)
        RVline = ChooseBestLines(RVscores2,RVlines2, 1)
        THline = ChooseBestLines(THscores2,THlines2, 1)
        BHline = ChooseBestLines(BHscores2,BHlines2, 1)
        lines3 = LVline + RVline + THline + BHline
        # img3 = PasteandPrintLines(lines3, img_rgb)
        #
        # images = [img_ext, img1, img2, img3]
        #
        # plt.figure(figsize=(20, 20))
        # for i in range(4):
        #     plt.subplot(1,4,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)

        Vcenterdist = computeCenterDistance(LVline[0][0],LVline[0][1],LVline[0][2],LVline[0][3],RVline[0][0],RVline[0][1],RVline[0][2],RVline[0][3])
        Hcenterdist = computeCenterDistance(THline[0][0],THline[0][1],THline[0][2],THline[0][3],BHline[0][0],BHline[0][1],BHline[0][2],BHline[0][3])
        if (230 <= Hcenterdist <= 240) and (480 <= Vcenterdist <= 500):
            # Determination of Selected Line Intersections, Yaw, and X-Y offsets
            x1,y1,x2,y2,x3,y3,x4,y4 = intersection(LVline, THline, RVline, BHline)
            yaw = averageYaw(x1, y1, x2, y2, x3, y3, x4, y4)

            x_deviation, y_deviation, x_mid, y_mid = centerOffset(x1, y1, x2, y2, x3, y3, x4, y4, scalingfactorX, scalingfactorY)
            euclid = math.sqrt(x_deviation**2 + y_deviation ** 2)

            yawAll.append(yaw)
            x_midAll.append(x_mid)
            y_midAll.append(y_mid)
            x_offsetAll.append(x_deviation)
            y_offsetAll.append(y_deviation)
            euclidAll.append(euclid)
            lines4.append(LVline + RVline + THline + BHline)


    if (len(euclidAll) % 2 == 0):
        #print("even!")
        sortedEuclid = euclidAll
        sortedEuclid.sort()
        m = int(len(sortedEuclid) / 2) - 1
        m_next = m + 1
        median1 = sortedEuclid[m]
        median2 = sortedEuclid[m_next]
        index = euclidAll.index(median1)
        index_next = euclidAll.index(median2)

        finalyaw = (yawAll[index] + yawAll[index_next]) / 2
        finalX = (x_offsetAll[index] + x_offsetAll[index_next]) / 2
        finalY = (y_offsetAll[index] + y_offsetAll[index_next]) / 2
        finalX_mid = (x_midAll[index] + x_midAll[index_next]) / 2
        finalY_mid = (y_midAll[index] + y_midAll[index_next]) / 2
        # print("Run:",j)
        # print("Yaw:", finalyaw)
        # print("X:", finalX)
        # print("Y:", finalY)
        # print("\n")
        img4 = PasteandPrintLines(lines4[index], img_rgb)
        # img5 = PasteandPrintLines(lines4[index_next], img_rgb)
        img6 = cv.circle(img4, (int(finalX_mid),int(finalY_mid)), radius=2, color=(0, 255, 0), thickness=-1)
        img_final = cv.circle(img6, (320,240), radius=2, color=(0, 0, 255), thickness=-1)
        #
        # images = [img4, img7]
        # plt.figure(figsize=(30, 30))
        # for i in range(2):
        #     plt.subplot(1,2,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)

    else:
        #print("odd!")
        euclidMedian = np.median(euclidAll)
        index = euclidAll.index(euclidMedian)
        finalyaw = yawAll[index]
        finalX = x_offsetAll[index]
        finalY = y_offsetAll[index]
        # print("Run:",j)
        # print("Yaw:", finalyaw)
        # print("X:", finalX)
        # print("Y:", finalY)
        # print("\n")
        img4 = PasteandPrintLines(lines4[index], img_rgb)
        img5 = cv.circle(img4, (int(x_midAll[index]),int(y_midAll[index])), radius=2, color=(0, 255, 0), thickness=-1)
        img_final = cv.circle(img5, (320,240), radius=2, color=(0, 0, 255), thickness=-1)

    cv.imwrite("capture/img_final.png", img_final)
        #
        # images = [img4, img6]
        # plt.figure(figsize=(20, 20))
        # for i in range(2):
        #     plt.subplot(1,2,i+1),plt.imshow(images[i],'gray',vmin=0,vmax=255)
    
    return send_file("capture/img_final.png")

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument(
        "--resolution",
        type=str,
        choices={"RES_720P", "RES_1080P", "RES_1440P", "RES_2160P"},
        default="RES_1080P",
    )
    return parser.parse_args()

def main():
    setup_flash()
    # Setup...
    args = parse_args()
    app.run(host=args.host, port=args.port)

if __name__ == "__main__":
    main()
