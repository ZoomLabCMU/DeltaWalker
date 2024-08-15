# Reference https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/

# Python code for Multiple Color Detection


import numpy as np
import cv2

# Capturing video through webcam
webcam = cv2.VideoCapture(0)

""" Loads camera matrix and distortion coefficients. """
# FILE_STORAGE_READ
cv_file = cv2.FileStorage('../camera.yml', cv2.FILE_STORAGE_READ)

# note we also have to specify the type to retrieve other wise we only get a
# FileNode object back instead of a matrix
camera_matrix = cv_file.getNode("K").mat()
dist_matrix = cv_file.getNode("D").mat()

cv_file.release()

size = (1920, 1080)

# Start a while loop
while (1):

    # Reading the video from the
    # webcam in image frames
    _, imageFrame = webcam.read()
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, size, 0, size)
    imageFrame = cv2.undistort(imageFrame, camera_matrix, dist_matrix, None, new_camera_matrix)

    # Convert the imageFrame in
    # BGR(RGB color space) to
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for red color and
    # define mask
    # This color is closer to a pink of the paints but can detetct red
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Set range for green color and
    # define mask
    green_lower = np.array([35, 100, 100], np.uint8)
    green_upper = np.array([85, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # Set range for blue color and
    # define mask
    blue_lower = np.array([100, 150, 0], np.uint8)
    blue_upper = np.array([140, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    # Set range for orange color and
    # define mask
    orange_lower = np.array([10, 100, 50], np.uint8)
    orange_upper = np.array([25, 255, 255], np.uint8)
    orange_mask = cv2.inRange(hsvFrame, orange_lower, orange_upper)

    # Set range for yellow color and
    # define mask
    yellow_lower = np.array([10, 72, 109], np.uint8)
    yellow_upper = np.array([33, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernel = np.ones((3, 3), "uint8")

    # For red color
    red_mask = cv2.dilate(red_mask, kernel)
    res_red = cv2.bitwise_and(imageFrame, imageFrame,
                              mask=red_mask)

    # For green color
    green_mask = cv2.dilate(green_mask, kernel)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=green_mask)

    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernel)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                               mask=blue_mask)

    # For orange color
    orange_mask = cv2.dilate(orange_mask, kernel)
    res_orange = cv2.bitwise_and(imageFrame, imageFrame,
                               mask=orange_mask)

    # For yellow color
    yellow_mask = cv2.dilate(yellow_mask, kernel)
    res_yellow = cv2.bitwise_and(imageFrame, imageFrame,
                               mask=yellow_mask)

    min_area = 500
    max_area = 800

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # print(area)
        if (area > min_area and area < max_area):
            (x,y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(imageFrame, center, radius, (0, 0, 255), 2)

        # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > min_area and area < max_area):
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(imageFrame, center, radius, (0, 255, 0), 2)

        # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > min_area and area < max_area):
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(imageFrame, center, radius, (255, 0, 0), 2)

    contours, hierarchy = cv2.findContours(orange_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > min_area and area < max_area):
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(imageFrame, center, radius, (0, 128, 255), 2)

    contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > min_area and area < max_area):
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(imageFrame, center, radius, (0, 255, 255), 2)


        # Program Termination
    # cv2.imshow("Multiple Color Detection in Real-TIme", blue_mask)
    # if cv2.waitKey(50) & 0xFF == ord('q'):
    #     cap.release()
    #     cv2.destroyAllWindows()
    #     break

    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    if cv2.waitKey(50) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break

