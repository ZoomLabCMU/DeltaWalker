import cv2
import numpy as np


# Green (Delta 1), Red (Delta 2), Orange (Delta 3), Blue (Delta 4)
# HSV - 0-180, 0-255, 0-255

green = {'name': 'green',
         'lower': np.array([50, 125, 70], np.uint8),
         'upper': np.array([100, 255, 255], np.uint8),
         'bgr': (0, 255, 0)}

red = {'name': 'red',
       'lower': np.array([140, 100, 150], np.uint8),  # 170, 120, 70
       'upper': np.array([180, 255, 255], np.uint8),
       'bgr': (0, 0, 255)}

orange = {'name': 'orange',
          'lower': np.array([0, 150, 150], np.uint8),
          'upper': np.array([25, 255, 255], np.uint8),
          'bgr': (0, 128, 255)}

orange = {'name': 'orange',
          'lower': np.array([0, 120, 150], np.uint8),  # 0, 150, 150
          'upper': np.array([25, 255, 255], np.uint8),
          'bgr': (0, 128, 255)}

blue = {'name': 'blue',
        'lower': np.array([100, 100, 50], np.uint8),
        'upper': np.array([120, 255, 255], np.uint8),
        'bgr': (255, 0, 0)}

colors = [green, red, orange, blue]


# Foot tracking helper function
def tracking(frame, idx, min_circle_area=3500, max_circle_area=7000, min_contour_area=500, max_contour_area=6000):
    # Convert BGR to HSV
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    data = np.zeros(8)
    black = (0, 0, 0)


    for i in np.arange(4):
        # Extract info from the color
        color = colors[i]
        lower = color['lower']
        upper = color['upper']
        bgr = color['bgr']

        # Create mask
        mask = cv2.inRange(hsvFrame, lower, upper)

        # Erosion Kernel
        erosion_kernel = np.ones((1, 1), 'uint8')
        mask = cv2.erode(mask, erosion_kernel)

        # Dilation kernel
        dilation_kernel = np.ones((5, 5), 'uint8')
        mask = cv2.dilate(mask, dilation_kernel)

        # Creating contour to track color
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        clean_frame = np.array(frame)

        # Find contour
        for pic, contour in enumerate(contours):
            contour_area = cv2.contourArea(contour)

            # Check that contour is within acceptable area range
            if (contour_area >= min_contour_area and contour_area <= max_contour_area):
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

                # Check that corresponding circle is foot sized
                circle_area = np.pi * radius**2

                if (circle_area >= min_circle_area and circle_area <= max_circle_area):
                    if found is True:  # re-annotate the frame
                        frame = cv2.circle(clean_frame, center, radius, bgr, 2)
                        frame = cv2.circle(frame, center, 5, black, -1)
                    else:  # annotate the blank frame
                        frame = cv2.circle(frame, center, radius, bgr, 2)
                        frame = cv2.circle(frame, center, 5, black, -1)

                    # Store data for this frame and color
                    data[i*2:(i+1)*2] = np.array([int(x), int(y)])
                    found = True

        if not found:
            print(f"No contour found for color {color['name']} at frame {idx}")
            # print(f"not found at {idx}")
            # for pic, contour in enumerate(contours):
            #     contour_area = cv2.contourArea(contour)
            #     (x, y), radius = cv2.minEnclosingCircle(contour)
            #     center = (int(x), int(y))
            #     radius = int(radius)
            #     circle_area = np.pi * radius ** 2
            #     print(f"num {pic} contour {contour_area} circle {circle_area}")
            #     mask = cv2.circle(mask, center, radius, (180, 255, 255), 2)
            # cv2.imshow("mask", mask)
            # cv2.waitKey(0)

    return data, frame


def contour_tuning(frame, idx, min_circle_area=3500, max_circle_area=7000, min_contour_area=1000, max_contour_area=6000):
    # Convert BGR to HSV
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    data = np.zeros(8)
    black = (0, 0, 0)

    for i in np.arange(4):
        # Extract info from the color
        color = colors[i]
        lower = color['lower']
        upper = color['upper']
        bgr = color['bgr']

        # Create mask
        mask = cv2.inRange(hsvFrame, lower, upper)

        # Erosion Kernel
        erosion_kernel = np.ones((1, 1), 'uint8')
        mask = cv2.erode(mask, erosion_kernel)

        # Dilation kernel
        dilation_kernel = np.ones((5, 5), 'uint8')
        mask = cv2.dilate(mask, dilation_kernel)

        # Creating contour to track color
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        found = False

        if i == 1:
            print(color['name'])

        # Find contour
        for pic, contour in enumerate(contours):
            contour_area = cv2.contourArea(contour)

            # Check that contour is within acceptable area range
            if (contour_area >= min_contour_area and contour_area <= max_contour_area):
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

                # Check that corresponding circle is foot sized
                circle_area = np.pi * radius**2

                if (circle_area >= min_circle_area and circle_area <= max_circle_area):
                    # Annotate frame
                    frame = cv2.circle(frame, center, radius, bgr, 2)
                    frame = cv2.circle(frame, center, 5, black, -1)

                    mask = cv2.circle(mask, center, radius, (180, 255, 255), 2)

                    # Store data for this frame and color
                    data[i*2:(i+1)*2] = np.array([int(x), int(y)])
                    if i == 1:
                        print(f"found contour {contour_area} circle {circle_area} at center {x}, {y}")

                    if found is True:
                        cv2.imshow("multiples", frame)
                        cv2.waitKey(0)
                        cv2.imshow("mask", mask)
                        cv2.waitKey(0)
                    found = True

        if found is False:
            print("not found")
            for pic, contour in enumerate(contours):
                contour_area = cv2.contourArea(contour)
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                circle_area = np.pi * radius ** 2
                print(f"num {pic} contour {contour_area} circle {circle_area}")
            cv2.imshow("mask", mask)
            cv2.waitKey(0)

        if not found:
            print(f"No contour found for color {color['name']} at frame {idx}")

    return data, frame


def mask_tuning(frame, idx, min_circle_area=3500, max_circle_area=7000, min_contour_area=1500, max_contour_area=5000):
    # Convert BGR to HSV
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    data = np.zeros(12)
    black = (0, 0, 0)

    # green, red, orange, blue
    i = 1
    # Extract info from the color
    color = colors[i]
    lower = color['lower']
    upper = color['upper']
    bgr = color['bgr']

    # Create mask
    mask = cv2.inRange(hsvFrame, lower, upper)

    # Erosion Kernel
    erosion_kernel = np.ones((1, 1), 'uint8')
    mask = cv2.erode(mask, erosion_kernel)

    # Dilation kernel
    dilation_kernel = np.ones((5, 5), 'uint8')
    mask = cv2.dilate(mask, dilation_kernel)

    return data, mask


# Load camera coefficients
def load_coefficients(path):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode('K').mat()
    dist_matrix = cv_file.getNode('D').mat()

    cv_file.release()
    return camera_matrix, dist_matrix

