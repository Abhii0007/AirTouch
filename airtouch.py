

from threading import Thread
import cv2,os,time
import ctypes
import win32api
import win32con
import mediapipe as mp
from colorama import init, Fore, Back, Style
from colorama import colorama_text,Fore


MOUSEEVENTF_MOVE = 0x0001
MOUSE_EVENT_LEFTDOWN = 0x0002
MOUSE_EVENT_LEFTUP = 0x0004
def Aircontrol():
    global x_1,y_1,lock,k_1,z_1

    print('To Activate Show the vicctory Sign')
    x_1=0
    y_1=0
    lock=0
    k_1=1
    z_1=0
    def cursor(m_1,n_1):
        ctypes.windll.user32.SetCursorPos(m_1,n_1)

    def Click_Down():
        win32api.mouse_event(MOUSE_EVENT_LEFTDOWN, 0, 0, 0, 0)
        print(Fore.CYAN+'-----Hold--------')
    def Click_Up():
        win32api.mouse_event(MOUSE_EVENT_LEFTUP, 0, 0, 0, 0)
        print(Fore.BLUE+'----Released-----')

    def double_click():
        win32api.mouse_event(MOUSE_EVENT_LEFTDOWN, 0, 0, 0, 0)
        time.sleep(0.01)
        win32api.mouse_event(MOUSE_EVENT_LEFTUP, 0, 0, 0, 0)
        time.sleep(0.05)
        win32api.mouse_event(MOUSE_EVENT_LEFTDOWN, 0, 0, 0, 0)
        time.sleep(0.01)
        win32api.mouse_event(MOUSE_EVENT_LEFTUP, 0, 0, 0, 0)
        
        

    def right_click():
        win32api.mouse_event(win32con.MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0)
        time.sleep(0.02)
        win32api.mouse_event(win32con.MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0)
        print(Fore.MAGENTA+'Right Clicked-----------------')

    def main_control():
        global x_1,y_1,lock,z_1,k_1
        cap = cv2.VideoCapture(0)
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(frame_rgb)
            #id=3(thumb point),id=4(thumb Tip)
            #id=5(Point finger bottom),id=6(Point finger 2nd bottom),id=7(Point finger 2nd top)
            if results.multi_hand_landmarks:
                for abhi in results.multi_hand_landmarks:
                    for id, lm in enumerate(abhi.landmark):
                        h, w, _ = frame.shape
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        m,n=int(640-cx)*3-260,int(cy)*2
                        if id==20: #Little finger tip
                            if lock==1:
                                Thread(target=cursor,args=(m,n)).start()
                        elif id ==4 :  # Thumb Tip Landmark
                            a=cx
                            b=cy
                        elif id ==8 :  # Index finger tip landmark
                            a1=cx
                            b1=cy
                            #print(a-a1)
                            if (b-b1)<=18 and (a-a1)<=18 :
                                if x_1==0:
                                    if lock==1:
                                        Thread(target=Click_Down).start()
                                        k_1=0
                                        
                                        
                                        #print(Fore.RED+'Hold------------------------------------',Fore.WHITE+'')
                                        x_1=1
                            else:
                                if k_1==0:

                                    Thread(target=Click_Up).start()
                                    #print(Fore.GREEN+'Free------------------------------------',Fore.WHITE+'')
                                    k_1=1
                                    x_1=0
                                else:
                                    pass
                        #-------------Victory Sign Identifier------------------------------
                        elif id==16: #Ring Finger tip
                            a2=cx
                            b2=cy
                            if (b-b2)<=18 and (a-a2)<=18:
                                if lock==0:
                                    lock=1
                                else:
                                    if z_1==0:
                                        Thread(target=right_click).start()
                                        z_1=1
                            else:
                                z_1=0
                        elif id==12:
                            a3=cx
                            b3=cy
                            if (b-b3)<=18 and (a-a3)<=18:
                                if lock==1:
                                    Thread(target=double_click).start()
                                    if y_1==0:
                                        print(Fore.RED+'Double Clicked Encountered--------------------')
                                        y_1=1
                                    
                                    
                                        
                            else:
                                y_1=0
                                
                        cv2.imshow('Finger Tracking', frame)
            if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
                break
        cap.release()
        cv2.destroyAllWindows()
    Thread(target=main_control).start()


Aircontrol()