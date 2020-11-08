import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def imshow(image):
    fig = plt.figure()
    ax = plt.axes([0,0,1,1])
    plt.imshow(image, interpolation="nearest", aspect='auto')
    DPI = fig.get_dpi()
    fig.set_size_inches(image.shape[1]/float(DPI),image.shape[0]/float(DPI))
    plt.axis("off")   # turns off axes
    plt.axis("tight")  # gets rid of white border
    plt.axis("image")  # square up the image instead of filling the "figure" space
    plt.show()
    return fig

#%% Image loading
image1 = cv2.imread("rectified1.png")
image2 = cv2.imread("rectified2.png")

gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

#%% Feature extraction
#feature_extractor = cv2.xfeatures2d.SIFT_create()
feature_extractor = cv2.ORB_create()
(keypoints1, descriptors1) = feature_extractor.detectAndCompute(gray1, None)
print("Image #1, kps: {}, descriptors: {}".format(len(keypoints1), descriptors1.shape))

(keypoints2, descriptors2) = feature_extractor.detectAndCompute(gray2, None)
print("Image #2, kps: {}, descriptors: {}".format(len(keypoints2), descriptors2.shape))

image1_with_keypoints = cv2.drawKeypoints(gray1, keypoints1, None)
image2_with_keypoints = cv2.drawKeypoints(gray2, keypoints2, None)

#image_with_keypoints = np.hstack((image1_with_keypoints, image2_with_keypoints))
image_with_keypoints = cv2.hconcat((image1_with_keypoints, image2_with_keypoints))
imshow(image_with_keypoints)

# Feature matching
bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

matches = bf.match(descriptors1, descriptors2)
matches = sorted(matches, key = lambda x:x.distance)

image_matches = cv2.drawMatches(gray1, keypoints1, gray2, keypoints2, matches[:50], None, flags=2)
fig = imshow(image_matches)
fig.savefig("matches.png")
