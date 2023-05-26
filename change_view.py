import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from sklearn.preprocessing import normalize


def find_homography(clicks1, clicks2):
    points1 = []
    All_A = []
    # print(clicks1.shape)
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
    # print("===>", Final_H.shape)
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
    # plt.show()
    # clicks1 = plt.ginput(numper_of_points)
    # print(clicks1)
    clicks1 = [(193.78571428571433, 818.2012987012987), (224.9545454545455, 608.6774891774891), (723.6558441558443, 606.9458874458874), (768.6774891774894, 856.2965367965368)]


    # clicks1 = [(795.2987987455929, 325.02789638737795), (774.8881043921972, 854.5720221115856), (148.96014422139933, 329.5636062436881), (189.7815329281907, 820.5541981892596)]
    # clicks1 = [(794.439482286421, 320.2067762169802), (722.0655976676385, 621.0107341637954), (223.36429896633967, 615.356524427953), (143.07452071737794, 324.7301440056542)]
    
    clicks1 = np.array(clicks1).astype(np.int64)
    # clicks1 = np.concatenate((clicks1, np.ones((numper_of_points, 1))), axis=1)

    # clicks_check = plt.ginput(numper_of_points)
    clicks_check = [[145, 326, 1], [179, 437, 1], [798, 322, 1], [714, 463, 1]]
    clicks_check = []
    image2 = cv2.imread('./view2.jpg')
    fig = plt.figure()
    ax = fig.add_subplot()
    img2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
    # ax.imshow(img2)
    # plt.show()
    # clicks2 = plt.ginput(numper_of_points)
    # print(clicks2)
    clicks2 = [(590.322510822511, 993.0930735930735), (424.0887445887447, 975.7770562770563), (436.2099567099567, 702.1839826839827), (573.0064935064936, 676.2099567099566)]

    # clicks2 = [(262.30689669145204, 685.6486719209029),  (579.9923313815668, 681.7586461900034), (181.91303158619849, 1039.6410134327452), (591.6624085742649, 998.1474056364852)]
    # clicks2 = [(263.04978354978357, 683.1363636363636), (437.9415584415585, 703.9155844155844), (427.5519480519481, 967.1190476190476), (183.3961038961039, 1031.1883116883116)]
    
    clicks2 = np.array(clicks2).astype(np.int64)
    # clicks2 = np.concatenate((clicks2, np.ones((numper_of_points, 1))), axis=1)

    Final_H = find_homography(clicks1, clicks2)
    print(Final_H)
    apply_perspective(img2, Final_H, clicks_check)

