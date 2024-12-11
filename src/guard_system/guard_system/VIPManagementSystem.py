import cv2
import os
import numpy
from matplotlib import pyplot as plt


class VIPManagementSystem:
    def __init__(self):
        pass

    def SIFT_feature_matching(self, frame, n=1000):
        img_path = ''
        if frame:
            t1 = cv2.imread(img_path, 0)
            t2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            sift = cv2.SIFT_create()

            kp1, des1 = sift.detectAndCompute(t1, None)
            kp2, des2 = sift.detectAndCompute(t2, None)

            f = cv2.drawKeypoints(t1, kp1, None, [0, 0, 255], flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            nf = cv2.drawKeypoints(t2, kp2, None, [255, 0, 0], flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            bf = cv2.BFMatcher()
            matches = bf.match(des1, des2)

            matches = sorted(matches, key=lambda x: x.distance)

            result = cv2.drawMatches(t1, kp1, t2, kp2, matches[:min(n, len(matches))], None, [0, 0, 255], flags=2)

            plt.imshow(result, interpolation='bicubic')
            plt.axis('off')
            plt.show()

            if len(matches) > 100:
                return True
            else:
                return False

def main():
    VIPManagementSystem.SIFT_feature_matching()

if __name__ == '__main__':
    main()