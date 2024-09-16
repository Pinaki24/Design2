import cv2
import numpy as np
import math

# Initialize the camera capture
cap = cv2.VideoCapture(0)  # 0 is typically the default camera on your laptop

if not cap.isOpened():
    print("Error: Unable to open the camera.")
else:
    # Loop to capture and process frames continuously
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Unable to capture video frame.")
            break
        
        # Get frame dimensions
        height, width, _ = frame.shape
        
        # Convert the frame to grayscale
        grayscale_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian Blur to reduce noise before edge detection
        blurred_image = cv2.GaussianBlur(grayscale_image, (5, 5), 0)

        # Apply Canny edge detection on the blurred image
        edges = cv2.Canny(blurred_image, 100, 200)

        # Apply the Probabilistic Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=10)

        # Create a copy of the original frame to draw lines on it
        line_image = np.copy(frame)

        # Calculate middle and top 1/3 y-axis of the frame
        middle_y = height // 2
        look_ahead_y = height // 3  # Define the top 1/3 Y-axis as the "look ahead point"

        # Function to calculate the angle relative to the vertical axis and the length of a line
        def calculate_line_attributes(x1, y1, x2, y2):
            # Calculate dx and dy (differences in x and y)
            dx = x1 - x2
            dy = y2 - y1

            if dx == 0:  # Vertical line
                angle = 90.0  # Perfectly vertical
            else:
                m = dy / dx

                # Calculate the angle relative to the vertical axis
                angle = math.degrees(math.atan2(dx, dy))  # Relative to the vertical axis

                # Adjust angle to account for direction (positive or negative)
                if m > 0:
                    angle = 180 + angle  # Tilt to the left
                elif m < 0:
                    angle = angle  # Tilt to the right

            # Calculate the length of the line
            length = math.sqrt((dx) ** 2 + (dy) ** 2)

            return angle, length

        # Store the attributes of lines near the middle and the "look ahead point" and their average x values
        lines_near_middle = []
        lines_near_look_ahead_point = []
        x_values_middle = []
        x_values_look_ahead = []

        # Draw **all** the lines on the image and calculate attributes
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                # Draw the line (for all lines)
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw all lines in blue

                # Calculate the midpoint of the line
                midpoint_y = (y1 + y2) // 2

                # Check if the midpoint is close to the middle y-axis of the frame
                if abs(midpoint_y - middle_y) < 20:  # Threshold of 20 pixels for the middle
                    # Draw the line (for lines near the middle)
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw middle lines in green

                    # Calculate angle and length
                    angle, length = calculate_line_attributes(x1, y1, x2, y2)

                    # Store the attributes of the line
                    lines_near_middle.append({'angle': angle, 'length': length})

                    # Calculate and store the average x-coordinate of the line
                    avg_x = (x1 + x2) / 2
                    x_values_middle.append(avg_x)

                    # Print the attributes for lines near the middle
                    print(f"Line near middle: Angle = {angle:.2f} degrees, Length = {length:.2f} pixels, Avg X = {avg_x:.2f}")

                # Check if the midpoint is close to the "look ahead point" (top 1/3 y-axis)
                elif abs(midpoint_y - look_ahead_y) < 20:  # Threshold of 20 pixels for the look ahead point
                    # Draw the line (for lines near the "look ahead point")
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw look ahead lines in red

                    # Calculate angle and length
                    angle, length = calculate_line_attributes(x1, y1, x2, y2)

                    # Store the attributes of the line
                    lines_near_look_ahead_point.append({'angle': angle, 'length': length})

                    # Calculate and store the average x-coordinate of the line
                    avg_x = (x1 + x2) / 2
                    x_values_look_ahead.append(avg_x)

                    # Print the attributes for lines near the look ahead point
                    print(f"Line near look ahead point: Angle = {angle:.2f} degrees, Length = {length:.2f} pixels, Avg X = {avg_x:.2f}")

        # Calculate the overall average x-value for lines near the middle
        if x_values_middle:
            overall_avg_x_middle = sum(x_values_middle) / len(x_values_middle)
            print(f"Overall average X value of lines near middle: {overall_avg_x_middle:.2f}")
        else:
            print("No lines detected near the middle of the screen.")

        # Calculate the overall average x-value for lines near the "look ahead point"
        if x_values_look_ahead:
            overall_avg_x_look_ahead = sum(x_values_look_ahead) / len(x_values_look_ahead)
            print(f"Overall average X value of lines near look ahead point: {overall_avg_x_look_ahead:.2f}")
        else:
            print("No lines detected near the look ahead point of the screen.")

        # Display the processed frame with all lines, lines near the middle, and lines near the look ahead point
        cv2.imshow('Hough Lines', line_image)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()