import cv2
import numpy as np


'''
    1. detect markers
    2. transform image
    3. detect objects
    4. pubish objects to motion planner
'''


def main():
    # Load the image
    image = cv2.imread('assets/tests/image2.png')

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Detect markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)



    # Draw detected markers
    if ids is not None:
    
        pts1 = np.empty((4,2))
        for id in ids:
            id = id[0]
            x = corners[id][0][0][0]
            y = corners[id][0][0][1]
            pts1[id,:] = np.float32([x,y])

        pts1 = order_points(pts1)
        pts2 = np.float32([[0,0],[297,0],[0,210],[297,210]])*6

        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(image, M, (int(297*6),int(210*6)))
        
        image = cv2.aruco.drawDetectedMarkers(image, corners, ids)

    dst = detect_and_mark_washers(dst)

    # Display the result
    image = cv2.resize(image,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
    cv2.imshow('Raw image', image)
    dst = cv2.resize(dst,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
    cv2.imshow('Transformed', dst)


    cv2.waitKey(0)
    cv2.destroyAllWindows()


def order_points(points):
    # Convert points to numpy array
    points = np.array(points)

    # Find the centroid of the points
    centroid = np.mean(points, axis=0)

    # Define top-left, top-right, bottom-left, and bottom-right quadrants based on centroid
    quadrants = [(np.sum((point - centroid) ** 2), point) for point in points]
    top_left = min(quadrants)[1]
    bottom_right = max(quadrants)[1]
    quadrants.remove((np.sum((top_left - centroid) ** 2), top_left))
    quadrants.remove((np.sum((bottom_right - centroid) ** 2), bottom_right))

    top_right = max(quadrants)[1]
    bottom_left = min(quadrants)[1]

    return np.float32([top_left, top_right, bottom_right, bottom_left])


def detect_and_mark_washers(image):
   
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours based on area
    min_area = 10000  # adjust as needed
    max_area = 30000  # adjust as needed
    washers = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            print(circularity)
            if circularity > 0.7:  # adjust circularity threshold as needed
                washers.append(contour)

    # Draw bounding boxes or circles around detected washers
    for washer_contour in washers:
        (x, y, w, h) = cv2.boundingRect(washer_contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # bounding rectangle
        cv2.circle(image, (x+int(w/2), y+int(h/2)), 5, (0, 0, 255), 5)  # bounding rectangle

    return image


if __name__ == "__main__":
    main()
