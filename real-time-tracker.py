import os
import sys
import cv2
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.widgets  import RectangleSelector
import pandas as pd
# from kcf import KCFTracker
# from mtf import MTFTracker

# kcf_t = cv2.TrackerMIL_create()
# kcf_t = MTFTracker()
def readTrackingData(filename):
    corners = []
    rect_file = open(filename, 'r')
    rect_coords = rect_file.readlines()[0].split(",")
    p1_x = int(float(rect_coords[0]))
    p1_y = int(float(rect_coords[1]))
    w = int(float(rect_coords[2]))
    h = int(float(rect_coords[3]))

    corners.append([p1_x, p1_y])
    corners.append([p1_x + w, p1_y])
    corners.append([p1_x, p1_y + h])
    corners.append([p1_x + w, p1_y + h])
    corners = np.array(corners).T
    return corners


def writeCorners(file_id, corners):
    # write the given corners to the file
    corner_str = ''
    for i in range(4):
        corner_str = corner_str + '{:5.2f}\t{:5.2f}\t'.format(corners[0, i], corners[1, i])
    file_id.write(corner_str + '\n')


def drawRegion(img, corners, color, thickness=1):
    # draw the bounding box specified by the given corners
    for i in range(4):
        p1 = (int(corners[0, i]), int(corners[1, i]))
        p2 = (int(corners[0, (i + 1) % 4]), int(corners[1, (i + 1) % 4]))
        cv2.line(img, p1, p2, color, thickness)


def initTracker(img, corners):
    # initialize your tracker with the first frame from the sequence and
    # the corresponding corners from the ground truth
    # this function does not return anything
    global old_frame
    global p0
    old_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    p0 = corners.T.astype(np.float32)
    pass


def updateTracker(img):
    # update your tracker with the current image and return the current corners
    # at present it simply returns the actual corners with an offset so that
    # a valid value is returned for the code to run without errors
    # this is only for demonstration purpose and your code must NOT use actual corners in any way
    global old_frame
    global p0
    frame_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (8,8),
                  maxLevel = 8,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 9, 0.01))
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_frame, frame_img, p0, None, **lk_params)
    old_frame = frame_img.copy()
    p0 = p1.copy()

    return p1.T

def line_select_callback(eclick, erelease):
    x1, y1 = eclick.xdata, eclick.ydata
    x2, y2 = erelease.xdata, erelease.ydata
    rect = plt.Rectangle( (min(x1,x2),min(y1,y2)), np.abs(x1-x2), np.abs(y1-y2) )
    ax.add_patch(rect)
    rect_file = open('initial-rect.txt', 'w')
    rect_coord = str(min(x1,x2)) + "," + str(min(y1,y2)) + "," + str(np.abs(x1-x2)) + "," + str(np.abs(y1-y2))
    rect_file.write(rect_coord)
    rect_file.close()

# tracker = cv2.TrackerMIL_create()

result_file = open('wam-arm-output.txt', 'w')
camera = cv2.VideoCapture("http://172.31.111.46:8080/video")
n_frames = 500
img_counter = 0
init_corners = []
ground_truth = []
init_img = 0
time_to_wait = 1
while img_counter < n_frames:
    ret, frame = camera.read()

    xxx = 0
    if img_counter == 0:
        init_img = frame
        # init_img = cv2.imread('./tracker_output_0/' + str(xxx + img_counter) +'_NoDraw.jpg')
        init_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # fig = plt.figure()
        # ax = fig.add_subplot()
        # ax.imshow(init_img)
        # rs = RectangleSelector(ax, line_select_callback,
        #                        drawtype='box', useblit=False, button=[1],
        #                        minspanx=5, minspany=5, spancoords='pixels',
        #                        interactive=True)
        # plt.show()

        fig = plt.figure()
        ax = fig.add_subplot()
        ax.imshow(init_img)
        clicks = plt.ginput(4)
        print(clicks)
        init_corners = [[clicks[0][0], clicks[1][ 0], clicks[2][ 0], clicks[3][ 0]],
                        [clicks[0][1], clicks[1][1], clicks[2][1], clicks[3][1]]]

        print(init_corners)
        init_corners = np.array(init_corners)
        # init_corners = readTrackingData('initial-rect.txt')
        print(init_corners)

        writeCorners(result_file, init_corners)

        initTracker(init_img, init_corners)

        fig = plt.figure()
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot()
        ax.imshow(np.array(init_img))

        tracker_corners = init_corners
        x_line = [tracker_corners[0, 0], tracker_corners[0, 1]]
        y_line = [tracker_corners[1, 0], tracker_corners[1, 1]]
        ax.plot(x_line, y_line, color='blue')
        x_line = [tracker_corners[0, 0], tracker_corners[0, 2]]
        y_line = [tracker_corners[1, 0], tracker_corners[1, 2]]
        ax.plot(x_line, y_line, color='blue')
        x_line = [tracker_corners[0, 3], tracker_corners[0, 1]]
        y_line = [tracker_corners[1, 3], tracker_corners[1, 1]]
        ax.plot(x_line, y_line, color='blue')
        x_line = [tracker_corners[0, 3], tracker_corners[0, 2]]
        y_line = [tracker_corners[1, 3], tracker_corners[1, 2]]
        pd.DataFrame(tracker_corners).to_csv("./tracker_output/" + str(img_counter) + "coord.csv")
        ax.plot(x_line, y_line, color='blue')
        # plt.show()
        canvas.draw()
        buf = canvas.buffer_rgba()
        image = np.asarray(buf)
        cv2.imwrite("./tracker_output/" + str(xxx + img_counter) + ".jpg", image)
        print("Saving....")
        print("./tracker_output/" + str(xxx + img_counter) + ".jpg")
        # kcf_t.init(init_img, init_corners)

    else:
        # tracker_corners = kcf_t.update(frame)
        # frame = cv2.imread('./tracker_output_0/' + str(xxx + img_counter) + '_NoDraw.jpg')

        tracker_corners = updateTracker(frame)
        writeCorners(result_file, tracker_corners)

        fig = plt.figure()
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot()
        ax.imshow(np.array(frame))


        # convert to a NumPy array
        x_line = [tracker_corners[0,0], tracker_corners[0,1]]
        y_line = [tracker_corners[1,0], tracker_corners[1,1]]
        ax.plot(x_line, y_line, color='blue')
        x_line = [tracker_corners[0, 0], tracker_corners[0, 2]]
        y_line = [tracker_corners[1, 0], tracker_corners[1, 2]]
        ax.plot(x_line, y_line, color='blue')
        x_line = [tracker_corners[0, 3], tracker_corners[0, 1]]
        y_line = [tracker_corners[1, 3], tracker_corners[1, 1]]
        ax.plot(x_line, y_line, color='blue')
        x_line = [tracker_corners[0, 3], tracker_corners[0, 2]]
        y_line = [tracker_corners[1, 3], tracker_corners[1, 2]]
        pd.DataFrame(tracker_corners).to_csv("./tracker_output/" + str(img_counter) + "coord.csv")
        ax.plot(x_line, y_line, color='blue')
        # plt.show()
        canvas.draw()
        buf = canvas.buffer_rgba()
        image = np.asarray(buf)

        cv2.imwrite("./tracker_output/" + str(xxx + img_counter) + ".jpg", image)
        cv2.imwrite("./tracker_output/" + str(img_counter) + "_NoDraw.jpg", frame)


    # img_name = "opencv_frame_{}.png".format(img_counter)
    # cv2.imwrite(img_name, frame)
    print(" written!", img_counter)
    img_counter += 1



camera.release()
cv2.destroyAllWindows()
result_file.close()
