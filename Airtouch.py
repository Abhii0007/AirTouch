from threading import Thread, Lock
import cv2
import time
import ctypes
import win32api
import win32con
import mediapipe as mp
import numpy as np

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

MOUSEEVENTF_MOVE = 0x0001
MOUSE_EVENT_LEFTDOWN = 0x0002
MOUSE_EVENT_LEFTUP = 0x0004
CLICK_THRESHOLD = 30  # Pixel distance for click detection
MIN_MOVEMENT_THRESHOLD = 2  # Reduced minimum pixel movement to update cursor
BASE_SMOOTHING_FACTOR = 0.3  # Adjusted base smoothing factor for smoother cursor movement
INTERPOLATION_STEPS = 10  # Number of interpolation steps between frames

class AirControl:
    def __init__(self):
        self.click_state = False
        self.lock = False
        self.right_click_state = False
        self.double_click_state = False
        self.last_click_time = 0
        self.last_toggle_time = 0
        self.cursor_x, self.cursor_y = 0, 0
        self.smoothed_x, self.smoothed_y = 0, 0
        self.target_x, self.target_y = 0, 0  # Target position for interpolation
        self.lock_obj = Lock()

    def move_cursor(self, x, y):
        with self.lock_obj:
            # Get the current position of the cursor
            current_x, current_y = self.cursor_x, self.cursor_y

            # Scale hand position to screen coordinates (adjust if necessary)
            screen_x = int(x)  # Direct hand position to screen coordinates
            screen_y = int(y)

            # Calculate the distance to the target position
            delta_x = screen_x - current_x
            delta_y = screen_y - current_y

            # Simulate smooth movement by moving the cursor step-by-step
            steps =  5  # Number of small steps to take
            for i in range(1, steps + 1):
                # Calculate intermediate positions for smooth motion
                intermediate_x = int(current_x + delta_x * (i / steps))
                intermediate_y = int(current_y + delta_y * (i / steps))

                # Move the cursor to the intermediate position
                ctypes.windll.user32.SetCursorPos(intermediate_x, intermediate_y)
                time.sleep(0.01)  # Small delay between each move to simulate real movement

            # Update the final cursor position after the movement is done
            self.cursor_x, self.cursor_y = screen_x, screen_y

    def left_click(self, pressed):
        if pressed:
            win32api.mouse_event(MOUSE_EVENT_LEFTDOWN, 0, 0, 0, 0)
            print("🖱️ Left Click: Pressed")
        else:
            win32api.mouse_event(MOUSE_EVENT_LEFTUP, 0, 0, 0, 0)
            print("🖱️ Left Click: Released")

    def double_click(self):
        win32api.mouse_event(MOUSE_EVENT_LEFTDOWN, 0, 0, 0, 0)
        win32api.mouse_event(MOUSE_EVENT_LEFTUP, 0, 0, 0, 0)
        time.sleep(0.05)
        win32api.mouse_event(MOUSE_EVENT_LEFTDOWN, 0, 0, 0, 0)
        win32api.mouse_event(MOUSE_EVENT_LEFTUP, 0, 0, 0, 0)
        print("🖱️ Double Click!")

    def right_click(self):
        win32api.mouse_event(win32con.MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0)
        time.sleep(0.02)
        win32api.mouse_event(win32con.MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0)
        print("🖱️ Right Click!")

    def track_hand(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FPS, 30)  # Set camera frame rate to 30 FPS
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.9,  # Increased confidence for better accuracy
            min_tracking_confidence=0.9    # Increased confidence for better accuracy
        )

        #start_time = time.time()  # Timeout start time
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)  # Flip horizontally for natural movement
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(frame_rgb)

            # Timeout after 30 seconds if no hand detected
            #if time.time() - start_time > 30:
            #    print("No hand detected. Exiting...")
            #    pass

            h, w, _ = frame.shape  # Get frame dimensions

            if results.multi_hand_landmarks:
                print("Hand landmarks detected")  # Debug print
                for hand_landmarks in results.multi_hand_landmarks:
                    landmark_dict = {id: (int(lm.x * w), int(lm.y * h)) for id, lm in enumerate(hand_landmarks.landmark)}

                    # Palm Center (Between middle finger base and ring finger base)
                    palm_x = (landmark_dict[9][0] + landmark_dict[13][0]) // 2
                    palm_y = (landmark_dict[9][1] + landmark_dict[13][1]) // 2

                    # Map hand position to screen coordinates
                    screen_x = palm_x * 3 - 250
                    screen_y = palm_y * 2

                    if self.lock:
                        self.move_cursor(screen_x, screen_y)

                    # Thumb and Index Finger tips
                    thumb_x, thumb_y = landmark_dict[4]
                    index_x, index_y = landmark_dict[8]

                    # Calculate Euclidean distance between thumb and index
                    distance = np.linalg.norm([thumb_x - index_x, thumb_y - index_y])

                    current_time = time.time()
                    if distance < CLICK_THRESHOLD:  # Click when fingers are close
                        if not self.click_state and (current_time - self.last_click_time) > 0.5:
                            self.left_click(True)
                            self.click_state = True
                            self.last_click_time = current_time
                    else:
                        if self.click_state:
                            self.left_click(False)
                            self.click_state = False

                    ring_x, ring_y = landmark_dict[16]
                    distance_right = np.linalg.norm([thumb_x - ring_x, thumb_y - ring_y])

                    if distance_right < CLICK_THRESHOLD:
                        if not self.right_click_state:
                            self.right_click()
                            self.right_click_state = True
                    else:
                        self.right_click_state = False

                    middle_x, middle_y = landmark_dict[12]
                    distance_double = np.linalg.norm([thumb_x - middle_x, thumb_y - middle_y])

                    if distance_double < CLICK_THRESHOLD:
                        if not self.double_click_state:
                            self.double_click()
                            self.double_click_state = True
                    else:
                        self.double_click_state = False

                    pinky_x, pinky_y = landmark_dict[20]
                    distance_pinky = np.linalg.norm([thumb_x - pinky_x, thumb_y - pinky_y])

                    if distance_pinky < CLICK_THRESHOLD and (current_time - self.last_toggle_time) > 1.0:
                        self.lock = not self.lock
                        self.last_toggle_time = current_time
                        print("🔒 Cursor Lock Toggled:", self.lock)

                    # Draw Hand Landmarks
                    for id, (cx, cy) in landmark_dict.items():
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                    # Display Status
                    cv2.putText(frame, f"Cursor: {'Active' if self.lock else 'Disabled'}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    cv2.putText(frame, f"Click: {'Holding' if self.click_state else 'Free'}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

            cv2.imshow("Hand Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'Q' to quit
                break
        
        cap.release()
        cv2.destroyAllWindows()

    def start(self):
        print("To Activate, show the victory sign ✌️")
        Thread(target=self.track_hand).start()

if __name__ == "__main__":
    air_control = AirControl()
    air_control.start()
