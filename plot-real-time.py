import matplotlib.pyplot as plt
import time
import os.path


img_counter = 30
wait = 1
img = None
plt.axis('off')
iteration = 0
while img_counter<90:
    # time.sleep(wait)
    path_to_file = "./tracker_output_0/" + str(img_counter) + ".jpg"
    if os.path.exists(path_to_file):
        print("plotting .. " + path_to_file)
        im = plt.imread(path_to_file)
        if img is None:
            img = plt.imshow(im)
            plt.pause(0.5)
        else:
            img.set_data(im)

        plt.title("frame: " + path_to_file)
        plt.pause(0.5)
        plt.draw()
        img_counter += 1

    iteration += 1

