import cv2
import numpy as np
from picarx_improved import Picarx
import time

def process_image(image):
    height, width = image.shape[:2]
    roi = image[height//2:, :]  # Use data from the bottom half of the screen

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours in the edges
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw all contours
    cv2.drawContours(roi, contours, -1, (0, 0, 255), 1)

    # Find the contour closest to the middle of the image
    middle = width // 2
    min_distance = float('inf')
    middle_contour = None
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            distance = abs(cX - middle)
            if distance < min_distance:
                min_distance = distance
                middle_contour = contour

    if middle_contour is not None:
        M = cv2.moments(middle_contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        center = (cX, cY)
        # Draw a circle at the center of the middle contour in green
        cv2.circle(roi, center, 5, (0, 255, 0), -1)
        return (center[0], center[1] + height//2), roi  # Adjust the y-coordinate of the center

    return None, roi

def control_robot(center, image_width):
    if center is not None:
        cX, cY = center
        deviation = cX - image_width // 2
        # Calculate the deviation as a proportion of the image width
        deviation_proportion = deviation / image_width
        # Convert the deviation proportion to a turning angle
        # The maximum turning angle is assumed to be 45 degrees
        turning_angle = deviation_proportion * 20
        print(turning_angle)
        return turning_angle
    return None
 
def main():
    px = Picarx()
    cap = cv2.VideoCapture(0)
    # Set desired frame width and height
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    px.set_cam_tilt_angle(-45)
    sample_rate = 0.05  # Set the desired sample rate here
    while True:
        ret, frame = cap.read()
        line_center, processed_image = process_image(frame)
        turning_angle = control_robot(line_center, frame.shape[1])
        if turning_angle is not None:
            # Use the turning angle to control the robot
            # The control function is assumed to take a turning angle in degrees
            px.set_dir_servo_angle(turning_angle)
            
        cv2.imshow('Line Following', processed_image)
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #px.forward(50)   
        time.sleep(sample_rate)  # Add this line to introduce a delay

    cap.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
    
