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

#%%
#image = mpimg.imread("test.png") # OBS! This one will normalize the pixel values
image = cv2.imread("test.png")
fig = imshow(image)

#%%
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
sift = cv2.xfeatures2d.SIFT_create()
(keypoints, descriptors) = sift.detectAndCompute(gray, None)
print("# kps: {}, descriptors: {}".format(len(keypoints), descriptors.shape))

image_with_keypoints = cv2.drawKeypoints(gray, keypoints, None)

fig = imshow(image_with_keypoints)
fig.savefig("sift.png")


#%%
surf = cv2.xfeatures2d.SURF_create()
(keypoints, descriptors) = surf.detectAndCompute(gray, None)
print("# kps: {}, descriptors: {}".format(len(keypoints), descriptors.shape))

image_with_keypoints = cv2.drawKeypoints(gray, keypoints, None)

fig = imshow(image_with_keypoints)
fig.savefig("surf.png")

#%%
kaze = cv2.KAZE_create()
(keypoints, descriptors) = kaze.detectAndCompute(gray, None)
print("# kps: {}, descriptors: {}".format(len(keypoints), descriptors.shape))

image_with_keypoints = cv2.drawKeypoints(gray, keypoints, None)

fig = imshow(image_with_keypoints)
fig.savefig("kaze.png")

#%%
akaze = cv2.AKAZE_create()
(keypoints, descriptors) = akaze.detectAndCompute(gray, None)
print("# kps: {}, descriptors: {}".format(len(keypoints), descriptors.shape))

image_with_keypoints = cv2.drawKeypoints(gray, keypoints, None)

fig = imshow(image_with_keypoints)
fig.savefig("akaze.png")

#%%
brisk = cv2.BRISK_create()
(keypoints, descriptors) = brisk.detectAndCompute(gray, None)
print("# kps: {}, descriptors: {}".format(len(keypoints), descriptors.shape))

image_with_keypoints = cv2.drawKeypoints(gray, keypoints, None)

fig = imshow(image_with_keypoints)
fig.savefig("brisk.png")

#%%
orb = cv2.ORB_create()
(keypoints, descriptors) = orb.detectAndCompute(gray, None)
print("# kps: {}, descriptors: {}".format(len(keypoints), descriptors.shape))

image_with_keypoints = cv2.drawKeypoints(gray, keypoints, None)

fig = imshow(image_with_keypoints)
fig.savefig("orb.png")
