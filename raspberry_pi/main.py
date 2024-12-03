import cv2
import math
import numpy as np
import time
import smbus

'''CONSTANTS'''
microstep = 16 # Set of stepper motor drivers
motor_speed = 50 * microstep

#PID coeff
Kp = 0.310  
Ki = 0.017 
Kd = 4.550 
max_step = 40 * microstep
min_step = 0
offset = 10 * microstep
neutral = max_step / 2


# TRAJECTORY
do_trajectory = False
tennis_ball = False
squash = False
line_trajectory = [
    [75,75],
    [-75,-75]
    ]
triangle_trajectory = [
    # First Side (from [100, -50] to [-100, -50])
    [100, -50], [77.78, -50], [55.56, -50], [33.33, -50], [11.11, -50],
     [-11.11, -50], [-33.33, -50], [-55.56, -50], [-77.78, -50], [-100, -50],

    # Second Side (from [-100, -50] to [0, 100])
    [-100, -50], [-88.89, -33.33], [-77.78, -16.67], [-66.67, 0],
     [-55.56, 16.67], [-44.44, 33.33], [-33.33, 50], [-22.22, 66.67],
     [-11.11, 83.33], [0, 100],

    # Third Side (from [0, 100] to [100, -50])
    [0, 100], [11.11, 83.33], [22.22, 66.67], [33.33, 50], [44.44, 33.33],
     [55.56, 16.67], [66.67, 0], [77.78, -16.67], [88.89, -33.33], [100, -50]
]


# Raspberry Pi I2C bus
bus = smbus.SMBus(1)  # Use I2C bus 1 (default for Raspberry Pi)
bus.close() # Reset the bus
time.sleep(0.1)
bus = smbus.SMBus(1)
# STM32 I2C address (shifted to 8-bit)
STM32_ADDRESS = 0x08 << 1  # 7-bit address shifted to 8-bit

def i2c_send(two_bytes):
    if two_bytes < 1:
        two_bytes = 1
    if two_bytes > 0xFFFF:
        two_bytes = 0xFFFF
    try:
        bus.write_byte(STM32_ADDRESS, (two_bytes & 0xFF))#(two_bytes & 0xFF))
        time.sleep(0.01)
        bus.write_byte(STM32_ADDRESS, (two_bytes >> 8))
        time.sleep(0.01)
    except Exception as e:
        print(f"I2C Failed: {e}")


if True:
    i2c_send(0xFFF2) # Reset Commanc

    # Send Speed
    i2c_send(0xFFF1)
    i2c_send(motor_speed)

    i2c_send(0xFFF0)
    i2c_send(max_step)
    i2c_send(max_step)
    i2c_send(max_step)

# Define a function to detect a yellow ball and also do PID
def detect_yellow_ball_and_PID(relative_x, relative_y):
    # Start capturing video from the webcam. If multiple webcams connected, you may use 1,2, etc.
    cap = cv2.VideoCapture(0)
    # *1 CAP_PROP_FPS sets the frame rate of the webcam to 30 fps here
    cap.set(cv2.CAP_PROP_FPS, 30)
   
    # just for first iteration
    start_time = time.time()
    sequence_count = 0
    if do_trajectory:
        center_x = int(relative_x + line_trajectory[0][0])
        center_y = int(relative_y + line_trajectory[0][1])
        kp = 0.20
        ki = 0.10
        kd = 3.6
    else:
        center_x = relative_x
        center_y = relative_y
   
    prevRoll = 0
    prevPitch = 0
   
    accum_error = np.zeros(2) # accumulated sum of past errors [I control]
    prev_error = np.zeros(2) # most recent error value [D control]
       
    is_ready = False
    change = False
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        if(time.time() - start_time) > 10:
            is_ready = True
        if(time.time() - start_time) > 2:
            change = True
        if change and do_trajectory and is_ready:
            change = False
            sequence_count += 1
            if sequence_count == len(line_trajectory):
                sequence_count = 0
            center_x = int(relative_x + line_trajectory[sequence_count][0])
            center_y = int(relative_y + line_trajectory[sequence_count][1])
            start_time = time.time()
        # *2 Set the image resolution to 480x480. Note increasing resolution increases processing power used, and may slow down video feed.
        frame = cv2.resize(frame, (480, 480))

        # Convert the frame from BGR to HSV color space to easily identify a colour
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # *3 Define the range of yellow color in HSV [Hue, Saturation, Value]
        if tennis_ball:
            ball_color_lower = np.array([30, 50, 40])  # Lower bounds for HSV (yellowish-green)
            ball_color_upper = np.array([80, 255, 160]) # [upper Hue, upper Saturation, upper
            kp = 0.36
        elif squash:
            ball_color_lower = np.array([150, 0, 20])  # Lower bounds for HSV (yellowish-green)
            ball_color_upper = np.array([250, 120, 80]) # [upper Hue, upper Saturation, upper
            kp = 0.36
        else:
            ball_color_lower = np.array([15, 150, 150]) # [lower Hue, lower Saturation, lower Value]
            ball_color_upper = np.array([30, 255, 255]) # [upper Hue, upper Saturation, upper Value]

        # Threshold the HSV image to get the colors defined above
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask
        mask = cv2.inRange(hsv, ball_color_lower, ball_color_upper)

        # Find contours in the mask
        # RETR_TREE retrieves all hierarchical contours and organizes them
        # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.circle(frame, (center_x, center_y), 2, (0, 0, 255), -1)
        # Find the index of the largest contour
        if contours:
            # Determines the larget contour size using the cv2.contour Area function
            largest_contour = max(contours, key=cv2.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            # * 4 Only consider large enough objects. If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
            if radius > 10:
                # Draw a yellow circle around the ball
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
                cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                #print(f"Ball position: ({int(x)}, {int(y)})")
               
                #################################
                #### compute the PID output #####
                #################################

                # Calculate errors [P,I,D control]
                #Roll is about x-axis (+ is down tilt), pitch is about y-axis (+ is left tilt)
                errors = np.array([center_y - y, center_x - x])
                accum_error += errors# * compTime
                #accum_error = np.clip(accum_error,-5000,5000)
                derivative = errors - prev_error
                prev_error = errors
               
                if abs(errors[0]) < 5 and abs(errors[1]) < 5:
                    change = True
               
                print(f"E: {errors[0]:8.2f} {errors[1]:8.2f}, D: {derivative[0]:8.2f} {derivative[1]:8.2f}, A: {accum_error[0]:8.2f} {accum_error[0]:8.2f}")

                # apply PID formula
                pid_output = Kp * errors + Kd * derivative + Ki * accum_error

                # Constrain output to 15 degrees (pitch and roll angles)
                pid_output = np.clip(pid_output,(-1*neutral),neutral)
                roll, pitch = pid_output
               
                h1_height = int(roll + neutral + 40)
                h2_height = int((((-1*pitch) - roll) / 2) + neutral + 40)
                h3_height = int(((pitch - roll) / 2) + neutral)
               
                #print(f"R: {roll}, P: {pitch}, H1: {h1_height},H2: {h2_height}, H3: {h3_height}")
               
                i2c_send(0xFFF0)
                i2c_send(h1_height)
                i2c_send(h2_height)
                i2c_send(h3_height)
           
        # Display the resulting frame
        cv2.imshow('frame', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            i2c_send(0xFFF0)
            i2c_send(0)
            i2c_send(0)
            i2c_send(0)
            #time.sleep(0.5)
            #i2c_send(0xFFF3)
            break

    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv2.destroyAllWindows()




def locate_lines():
    # Start capturing video from the webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Set the frame rate
    cap.set(cv2.CAP_PROP_FPS, 30)
   
    x_coords = []
    y_coords = []

    for a in range(20):
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Resize the frame to speed up processing
        frame_ = cv2.resize(frame, (480, 480))
        frame_resized = frame_[120:370, 120:370]

        # Convert to grayscale
        gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

        # Apply edge detection (Canny)
        edges = cv2.Canny(gray, 50, 200, apertureSize=3)

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=30)

        if lines is not None:
            lines = np.squeeze(lines)  # Remove single-dimensional entries
            for line in lines:
                if len(line) == 4:  # Ensure line has exactly 4 values
                    x1, y1, x2, y2 = line
                    cv2.line(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Step 1: Calculate the angle for each line
        angles = []
        if lines is not None:
            for line in lines:
                if len(line) == 4:
                    x1, y1, x2, y2 = line
                    angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180  # Normalize angle between 0-180 degrees
                    angles.append((angle, line))  # Store both angle and the line

        # Step 2: Filter out duplicate or near-parallel lines
        filtered_lines = []
        unique_angles = []
       
        #print(angles)

        for angle, line in angles:
            # Check if the angle is close to any already added angle (to filter parallel lines)
            is_duplicate = False
            for existing_angle in unique_angles:
                if abs(angle - existing_angle) < 10:  # Lines are considered parallel if their angle difference is small
                    is_duplicate = True
                    break
            #print(is_duplicate)
            if not is_duplicate:
                filtered_lines.append(line)
                unique_angles.append(angle)

        # Step 3: If we have 3 distinct lines, check if their angles are 120 degrees apart
        if filtered_lines:
            # Draw the lines on the frame
            for line in filtered_lines:
                x1, y1, x2, y2 = line
                #cv2.line(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
           
            # Compute the intersection of the first two lines (as the center)
            try:
                x1, y1, x2, y2 = filtered_lines[0]
                x3, y3, x4, y4 = filtered_lines[1]
            except:
                return [120, 120]

            # Compute the intersection point of two lines using determinant method
            denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if denominator == 0:
                continue  # Lines are parallel, skip to the next iteration

            intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
            intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

            # Visualize the center on the frame
            #cv2.circle(frame_resized, (int(intersect_x), int(intersect_y)), 5, (0, 0, 255), -1)

            # Return the center coordinates as relative_x and relative_y
            relative_x = intersect_x
            relative_y = intersect_y
            #print(f"Center of the platform: ({relative_x}, {relative_y})")

            # Return the center coordinates
            x_coords.append(relative_x)
            y_coords.append(relative_y)

        # Show the processed frame
        cv2.imshow('Detected Lines', frame_resized)

        # Wait for key press, break the loop if ESC key (ASCII 27) is pressed
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
            break
    # Release the capture and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    return x_coords,y_coords
   
def remove_outliers_and_average(data):
    # Convert input to numpy array for easier processing
    data = np.array(data)
   
    # Calculate Q1 (25th percentile) and Q3 (75th percentile)
    Q1 = np.percentile(data, 25)
    Q3 = np.percentile(data, 75)
   
    # Calculate IQR (Interquartile Range)
    IQR = Q3 - Q1
   
    # Define outlier thresholds
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
   
    # Filter out data points that are outside the bounds
    filtered_data = data[(data >= lower_bound) & (data <= upper_bound)]
   
    # Calculate and return the average of the filtered data
    return int(np.mean(filtered_data))



x_coords, y_coords = locate_lines()

# Get average and remove outliers:
relative_x = remove_outliers_and_average(x_coords) + 120
relative_y = remove_outliers_and_average(y_coords) + 120

print(relative_x)
print(relative_y)

# Call the function to detect the yellow ball
detect_yellow_ball_and_PID(relative_x, relative_y)

bus.close()