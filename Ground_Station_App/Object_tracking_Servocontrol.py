import cv2
import numpy as np
import time

# Initialize video capture
cap = cv2.VideoCapture(0)

# Global variables
selection_start = None
selection_end = None
roi_selected = False
track_window = None
roi_hist = None
term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
fps_start_time = time.time()
fps_counter = 0
reset_tracking = False

# Servo motor parameters (adjust as needed)
SERVO_RANGE_X = 180  # Servo X-axis range (degrees)
SERVO_RANGE_Y = 180  # Servo Y-axis range (degrees)
FRAME_WIDTH = 640    # Frame width
FRAME_HEIGHT = 480   # Frame height

# Function to calculate servo commands
def calculate_servo_commands(object_position):
    center_x = FRAME_WIDTH // 2
    center_y = FRAME_HEIGHT // 2

    # Calculate the angle differences from the center of the frame
    delta_x = (object_position[0] - center_x) / center_x
    delta_y = (object_position[1] - center_y) / center_y

    # Calculate servo commands based on the angle differences
    servo_command_x = int(delta_x * SERVO_RANGE_X)
    servo_command_y = int(delta_y * SERVO_RANGE_Y)

    return servo_command_x, servo_command_y

# Mouse callback function for object selection
def select_object(event, x, y, flags, param):
    global selection_start, selection_end, roi_selected, track_window
    
    if event == cv2.EVENT_LBUTTONDOWN:
        selection_start = (x, y)
        roi_selected = False
    elif event == cv2.EVENT_LBUTTONUP:
        selection_end = (x, y)
        roi_selected = True
        x, y = min(selection_start[0], selection_end[0]), min(selection_start[1], selection_end[1])
        w, h = abs(selection_start[0] - selection_end[0]), abs(selection_start[1] - selection_end[1])
        track_window = (x, y, w, h)

# Create a window and set mouse callback
cv2.namedWindow('Select Object')
cv2.setMouseCallback('Select Object', select_object)

while True:
    ret, frame = cap.read()

    if ret:
        # Display FPS
        if not reset_tracking:
            fps_counter += 1
            if time.time() - fps_start_time >= 1:
                fps = fps_counter / (time.time() - fps_start_time)
                fps_counter = 0
                fps_start_time = time.time()

        if reset_tracking:
            reset_tracking = False
            track_window = None
            roi_hist = None

        if roi_selected:
            # Extract the ROI if it's within frame boundaries
            if track_window is not None and \
                    track_window[0] >= 0 and track_window[1] >= 0 \
                    and track_window[0] + track_window[2] <= frame.shape[1] \
                    and track_window[1] + track_window[3] <= frame.shape[0]:
                roi = frame[track_window[1]:track_window[1] + track_window[3],
                            track_window[0]:track_window[0] + track_window[2]]

                # Convert ROI to HSV color space
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                # Create a mask between the HSV bounds
                mask = cv2.inRange(hsv_roi, np.array((0., 60., 32.)), np.array((180., 255., 255.)))

                # Histogram to target on each frame
                roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])

                # Normalize the histogram
                cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

                # Reset roi_selected flag
                roi_selected = False

        if roi_hist is not None:
            # Convert frame to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Calculate the back projection
            dst = cv2.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)

            # Apply CAMShift to get the new location
            if track_window is not None:
                ret, track_window = cv2.CamShift(dst, track_window, term_crit)

                # Draw the window on the frame
                pts = cv2.boxPoints(ret)
                pts = np.intp(pts)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

                # Display the position of the tracked object
                cx, cy = ret[0][0], ret[0][1]
                cv2.putText(frame, f"Position: ({cx}, {cy})", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"FPS: {int(fps)}", (frame.shape[1] - 100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Calculate servo commands
                servo_command_x, servo_command_y = calculate_servo_commands((cx, cy))

                # Print servo commands (replace with actual servo control commands)
                print(f"Servo X command: {servo_command_x}, Servo Y command: {servo_command_y}")

        # Show the frame
        cv2.imshow('Select Object', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            reset_tracking = True

    else:
        break

cap.release()
cv2.destroyAllWindows()
