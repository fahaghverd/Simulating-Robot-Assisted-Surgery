import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from sklearn.preprocessing import normalize
from numpy import linalg as LA

def find_homography(clicks1, clicks2):
    points1 = []
    All_A = []
    for i in range(len(clicks1)):
        xi = np.concatenate((clicks2[i, :], [1]))
        row1 = [0, 0, 0]
        row1.extend(-1 * xi)
        row1.extend(clicks1[i][1] * xi)
        row2 = []
        row2.extend(xi)
        row2.extend([0, 0, 0])
        row2.extend(-clicks1[i][0] * xi)

        All_A.append(row2)
        All_A.append(row1)

    All_A = np.array(All_A)
    u, s, vh = np.linalg.svd(All_A, full_matrices=True)
    h = vh[-1, :]
    Final_H = h.reshape((3, 3))
    Final_H = Final_H / Final_H[-1][-1]
    return Final_H

def apply_perspective(image, homography, check_points=[]):
    warped = cv2.warpPerspective(image, homography, (image.shape[1], image.shape[0]))
    warped = warped.astype(np.uint8)
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.imshow(warped)

    if check_points != []:
        print(homography.shape)

        check_points = np.array(check_points)
        print(check_points.shape)
        converted_points = (homography @ check_points.T).T
        print("======>")
        print(converted_points)
        for i in range(converted_points.shape[0]):
            converted_points[i][:] /= converted_points[i][-1]

        X = converted_points[:][0]
        Y = converted_points[:][1]
        ax.scatter(X, Y, s=5, c="r")
        print(converted_points)
        # converted_points[:, :] = converted_points[:, :] / converted_points[:, :]


    plt.show()

if __name__ == '__main__':
    # numper_of_points = int(input("Please enter the number of points to select: "))
    numper_of_points = 4
    image1 = cv2.imread('./view1.jpg')
    fig = plt.figure()
    ax = fig.add_subplot()
    img1 = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)

    # ax.imshow(img1)
    # # plt.show()
    # clicks1 = plt.ginput(1)
    # print(clicks1)

    clicks1 = [(796.383116883117, 322.9632034632034), (721.9242424242425, 468.4177489177488), (727.1190476190477, 624.2619047619047), (777.335497835498, 854.5649350649351), (147.03246753246754, 331.621212121212), (188.59090909090912, 445.90692640692635), (224.9545454545455, 617.3354978354978), (190.32251082251082, 823.3961038961038)]
    clicks1 = np.array(clicks1).astype(np.float64)

    file = open("Reg-Poses2.file", "r")
    robot_pos = []
    x = []
    for i in file.readlines():
        if "position" in i:
            x = []
        elif "x:" in i:
            x.append(float(i.split(":")[1][1:]))
        elif "y:" in i:
            x.append(float(i.split(":")[1][1:]))
        elif "z:" in i:
            # x.append(float(i.split(":")[1][1:]))
            # x.append(1)
            robot_pos.append(x)


    robot_pos = np.array(robot_pos).astype(np.float64)

    robot_to_image_H = find_homography(clicks1[1:], robot_pos[1:])
    changed_check = robot_to_image_H @ np.concatenate((robot_pos[0] , np.ones((1))))
    changed_check /= changed_check[-1]
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.imshow(img1)
    ax.scatter([384], [434], s=10, c="r")
    plt.show()

    image_to_robot_H = find_homography(robot_pos[:], clicks1[:])
    # changed_check = image_to_robot_H @ np.concatenate((clicks1[0], np.ones((1))))
    changed_check = image_to_robot_H @ np.array([434, 454, 1])
    print("===")
    print(changed_check)
    changed_check /= changed_check[-1]
    print(changed_check)
    error = LA.norm(changed_check[:2]-robot_pos[0])
    print(error)
    # apply_perspective(img2, Final_H)

