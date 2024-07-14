import cv2 as cv


def main():
    CAMERA_IP_ADDRESS = "http://10.42.0.13:4747/video"
    cap = cv.VideoCapture(CAMERA_IP_ADDRESS)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        cv.waitKey(1)
        cv.imshow("frame", frame)
    return



if __name__ == "__main__":
    main()