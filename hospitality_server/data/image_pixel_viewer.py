import os, sys
for x in reversed(range(len(sys.path))):
    if "ros" in sys.path[x]:
        sys.path.pop(x)

import cv2
from matplotlib import pyplot as plt

image = cv2.imread(sys.argv[1])
plt.imshow(image)
plt.show()