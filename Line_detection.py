import cv2
import numpy as np
import math

# Function to calculate the angle relative to the vertical axis and the length of a line
def calculate_line_attributes(x1, y1, x2, y2):
    # Calculate dx and dy (differences in x and y)
    dx = x1 - x2
    dy = y2 - y1

    if dx == 0:  # Vertical line
        angle = 90.0  # Perfectly vertical
    else:
        m = dy / dx
        angle = math.degrees(math.atan2(dx, dy))
        if m > 0:
            angle = 180 + angle  # Tilt to the left
        elif m < 0:
            angle = angle  # Tilt to the right

    # Calculate the length of the line
    length = math.sqrt(dx ** 2 + dy ** 2)

    return angle, length

# Function to compute x-intersection at a given y_target (middle_y)
def compute_x_intersection(x1, y1, x2, y2, y_target):
    if y2 - y1 == 0:
        return None  # No intersection if the line is horizontal and not at y_target
    if x2 - x1 != 0:
        m = (y2 - y1) / (x2 - x1)
        x_intersection = x1 + (y_target - y1) / m
    else:
        x_intersection = x1  # Vertical line

    if min(y1, y2) <= y_target <= max(y1, y2):
        if min(x1, x2) <= x_intersection <= max(x1, x2):
            return x_intersection
    return None

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

        # Preprocessing
        grayscale_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(grayscale_image, (5, 5), 0)
        edges = cv2.Canny(blurred_image, 100, 200)

        # Line detection
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)
        line_image = np.copy(frame)

        # Reference points
        middle_x = width // 2
        middle_y = height // 2
        look_ahead_y = height // 3

        # Draw reference markers (Middle Cross)
        cv2.line(line_image, (middle_x, middle_y - 10), (middle_x, middle_y + 10), (0, 0, 255), 2)
        cv2.line(line_image, (middle_x - 10, middle_y), (middle_x + 10, middle_y), (0, 0, 255), 2)

        lines_near_middle = []
        x_values_middle = []
        x_intersections = []

        # Process detected lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Draw all lines in blue
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                midpoint_y = (y1 + y2) // 2

                if abs(midpoint_y - middle_y) < 50:
                    # Lines near the middle
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    angle, length = calculate_line_attributes(x1, y1, x2, y2)
                    lines_near_middle.append({'angle': angle, 'length': length})
                    avg_x = (x1 + x2) / 2
                    distance_to_middle = abs(avg_x - middle_x)
                    x_values_middle.append(avg_x)

                    # Compute x-intersection at middle_y
                    x_intersection = compute_x_intersection(x1, y1, x2, y2, middle_y)
                    if x_intersection is not None:
                        x_intersections.append(x_intersection)
                    else:
                        continue  # Skip if there's no valid intersection

                    print(f"Line near middle: Angle = {angle:.2f} degrees, Length = {length:.2f} pixels, Distance to middle = {distance_to_middle:.2f} px")

        # Calculate and display overall average intersection point
        if x_intersections:
            overall_avg_x_intersection = sum(x_intersections) / len(x_intersections)
            distance_to_middle = abs(overall_avg_x_intersection - middle_x)
            print(f"Overall average X intersection at middle_y: {overall_avg_x_intersection:.2f}")
            print(f"Distance between middle cross and detected line: {distance_to_middle:.2f} pixels")

            # Draw line from middle cross to intersection point
            cv2.line(line_image, (int(middle_x), int(middle_y)), (int(overall_avg_x_intersection), int(middle_y)), (0, 255, 0), 2)
            # Optionally, draw a circle at the intersection point
            cv2.circle(line_image, (int(overall_avg_x_intersection), int(middle_y)), 5, (0, 255, 255), -1)

            # Display this distance on the image
            cv2.putText(line_image, f"Distance: {distance_to_middle:.2f} px", 
                        (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            print("No lines detected near the middle of the screen.")

        # Display the processed frame
        cv2.imshow('Hough Lines', line_image)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
