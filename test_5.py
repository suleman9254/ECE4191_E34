import cv2

# Read the image
img = cv2.imread('test.jpg')

# Convert to YCrCb color space
ycrcb_img = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)

# Equalize the histogram of the Y channel
ycrcb_img[:, :, 0] = cv2.equalizeHist(ycrcb_img[:, :, 0])

# Convert back to BGR color space
equalized_img = cv2.cvtColor(ycrcb_img, cv2.COLOR_YCrCb2BGR)

cv2.imwrite('test2.jpg', equalized_img)

