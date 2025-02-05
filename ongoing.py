import keyboard # problematic
from djitellopy import tello
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import threading
import time
from datetime import datetime # for naming of pictures. Not very significant

import Send_to_server
import april_tags

saved_qr = []
phase = 1 # phase 1 is start take off and localisation, phase 2 is linefollower to first cabinet 

only_frames = None
imgTH = None
frame_export = None
wait_flag = False

CENTER = np.array([240, 180])
TARGET_Y = 150
BOUND_C = np.array([1 , 480])
ABS_PATH = "/Users/arman/IBM project/QRtesting4/"

# night testing
# blue_lower_limit = np.array([86 , 43 , 30])
# blue_upper_limit = np.array([103 , 138 , 188])

# red_move_lower_limit = np.array([0 , 255 , 121]) 
# red_move_upper_limit = np.array([5 , 255 , 255])

# green_move_lower_limit = np.array([30 , 112 , 122]) 
# green_move_upper_limit = np.array([34 , 129 , 163]) 

#projector testing
red_move_lower_limit = np.array([140 , 139 , 143]) 
red_move_upper_limit = np.array([179 , 255 , 255])
        # Set the lower and upper HSV limits
blue_lower_limit = np.array([114 , 92 , 133])
blue_upper_limit = np.array([125 , 255 , 255])

green_move_lower_limit = np.array([42 , 150 , 125])
green_move_upper_limit = np.array([64 , 255 , 255])

def run_video(drone):
    global saved_qr , only_frames , imgTH , frame_export
    
    def qr_DetectionAndSaving(img): # same qr code wont be scan. 
        global wait_flag
        if decode(img) != []:  
            qr_codes = decode(img)
            for qr_code in qr_codes:
                x , y , w, h = qr_code.rect
                if qr_code.data not in saved_qr: # basically only QR that aernt scan can be used
                    cropped_img = img[y:y+h, x:x+w]  # Crop the image to the QR code area
                    if np.sum(cropped_img) != 0: #to check if empty
                        name_QR_local = f"{datetime.now().strftime("%Y%m%d_%H%M%S")}qr_code.jpg"
                        cv2.imwrite(ABS_PATH + name_QR_local, cropped_img)  # Save the cropped image
                        saved_qr.append(qr_code.data)
                        qr_name_ibm = "QR" + str(len(saved_qr)) 
                        wait_flag = True
                        print("Sending")
                        Send_to_server.IBM_server(qr_name_ibm , name_QR_local , ABS_PATH + name_QR_local)
                        wait_flag = False
                        

    def Thrimage(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        (T, threshInv) = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return threshInv

    print("nus is tdy")
    while True:
        frame_for_processing = drone.get_frame_read().frame #getting the drone cam feed
        if phase == 1 or phase == 2:
            # Lfollowerstuff
            ReshapeInfo = np.array([480, 360]) # height , width, 
            frame_for_processing = cv2.resize(frame_for_processing, (ReshapeInfo[1], ReshapeInfo[0]))
            frame_for_processing = np.rot90(frame_for_processing,1, (1,0))

            imgTH = Thrimage(frame_for_processing) # threshold img for floor
            only_frames = frame_for_processing.copy()
            # cv2.imshow("Threshold floor" ,imgTH)
            # Lfollowerstuff
        if phase == 3 or phase == 4:

            ReshapeInfo = np.array([480 , 640]) #height ,  width
            frame_for_processing = cv2.resize(frame_for_processing, (ReshapeInfo[1], ReshapeInfo[0]))
            frame_export = cv2.cvtColor(frame_for_processing, cv2.COLOR_RGB2HSV) #hell looking ass
            frame_for_processing = cv2.cvtColor(frame_for_processing, cv2.COLOR_RGB2BGR)
            only_frames = frame_for_processing.copy() 
            qr_DetectionAndSaving(frame_for_processing)
        cv2.imshow("RGB Frame" , only_frames)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            break

def check_force_end(drone):
    print("forced end")
    drone.land()
    cv2.destroyAllWindows()
    exit()

def init_drone():
    me = tello.Tello()
    
    # Will slow down ur program alot
    # keyboard.add_hotkey('q' ,check_force_end , args = (me,) ) # ensures that the drone will emergency land no matter wat

    me.connect()
    me.streamon()
    me.set_video_direction(me.CAMERA_DOWNWARD)

    print(f"Bat {me.get_battery()}")
    vid_thread = threading.Thread(target = run_video , args = (me , ) , daemon=True)
    vid_thread.start()
    while only_frames is None or imgTH is None: # check if all frames are ready to be read 
        continue
    me.takeoff()

    return me

def localisation_aprilTag(ideal_aprilId):
    global only_frames , me
    # phase_1
    while True:
        april_id , go_xyz_x, go_xyz_y, distance = april_tags.detect(only_frames , me)
        april_tags.move_to(ideal_aprilId , me , only_frames)
        if april_id == ideal_aprilId and distance < 70:
            me.send_rc_control(0 , 0, 0,0)
            break

def FloorLineFollowerFSM(me, LRSpeed , FBSpeed):
    global imgTH , CENTER , TARGET_Y , BOUND_C 
    def LineFollowerAngle(BWImg, Center, TargetY, BoundC):
        TargetNode = np.array(BWImg[TargetY, BoundC[0]:BoundC[1]]) #(1, 320)
        TargetArray = np.nonzero(TargetNode)
        if TargetArray[0].size != 0:
            minCol = np.min(TargetArray)
            maxCol = np.max(TargetArray)
        else:
            minCol = 0
            maxCol = 0
        TargetX = (minCol + maxCol)/2 + BoundC[0]
        diffCol = TargetX - Center[0]
        # radian = math.atan2((TargetX-Center[0]), -(TargetY-Center[1]))
        # angle = math.degrees(radian)
        return TargetX, diffCol 
    state = 10
    while state != 13:
        tag_id, go_xyz_x, go_xyz_y, distance = april_tags.detect(only_frames , me)
        TargetX, diffCol = LineFollowerAngle(imgTH, CENTER , TARGET_Y , BOUND_C)
        match state:
            case 10:
                me.send_rc_control(0, FBSpeed, 0, 0)
                if diffCol > 5:
                    state = 11
                elif diffCol < -5:
                    state = 12
                elif tag_id == 294 or tag_id == 337:
                    state = 13
            case 11:
                me.send_rc_control(LRSpeed, FBSpeed, 0, 0)
                
                if 5 >= diffCol >= -5:
                    state = 10
                elif diffCol < -5:
                    state = 12
                elif tag_id == 294 or tag_id == 337:
                    state = 13
            case 12:
                me.send_rc_control(0, FBSpeed, 0, 0)
                if 5 >= diffCol >= -5:
                    state = 10
                elif diffCol > 5:
                    state = 11
                elif tag_id == 294 or tag_id == 337:
                    state = 13
            case 13:
                break

def blue_color_follower(frame_display , mask , phase , wait_flag , drone):
    # get the bounding box from the mask image
    bound_box = cv2.boundingRect(mask)

    def movement_guide():
        if h == 0 and w == 0:
            cv2.putText(frame_display , f"no boxes?" , (0,500) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,255,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            forward_back = -8
        elif h > 75: # might have to increase repeatedly
            cv2.putText(frame_display , f"move back" , (0,500) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,255,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            forward_back = -8

        elif h < 55:
            cv2.putText(frame_display , f"move forward" , (0,500) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,255,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            forward_back = 8
        else: 
            cv2.putText(frame_display , f"sweet spot" , (0,500) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,255,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            forward_back = 0

        # this should be fine
        if y > 320:
            cv2.putText(frame_display , f"lower" , (200 ,500) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,255,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            up_down = -8
        elif  y  < 300:
            cv2.putText(frame_display , f"higher" , (200 ,500) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,255,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            up_down = 8
        else:
            cv2.putText(frame_display , f"H OK" , (200 ,500) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,255,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            up_down = 0
        return forward_back , up_down
    
    yaw_angle = drone.get_yaw()
    turn_angle = 0
    if yaw_angle >=5:
        turn_angle = 8
    elif  yaw_angle <= -5:
        turn_angle = -8

    cv2.putText(frame_display , f"yaw angle: {yaw_angle}" , (400,200) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
    if wait_flag == False:
        if bound_box is not None: # when there is a specified color
            x, y, w, h = bound_box
            cv2.putText(frame_display , f"(x: {x} , y: {y}) of Width: {w} and Height: {h}" , (0,200) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
            cv2.rectangle(frame_display, (x, y), (x + w, y + h), (0, 150, 255), 2) #making the bounding box
            forward_back = 0
            up_down = 0
            left_right = 0
            if phase % 2 == 0: # assume that u always start at the right of the cabinet to move left first
                forward_back , up_down  = movement_guide()
                left_right = 10
            elif phase % 2 != 0:
                forward_back , up_down  = movement_guide()
                left_right = -10
            return left_right , forward_back , up_down , turn_angle
        else:
            print("why tf1") 
            return 0 , -10, 0, turn_angle
    elif wait_flag == True:
        return 0 , 0, 0, turn_angle

def color_checker(frame_display , mask):
    # create a mask for the specified color range
    # get the bounding box from the mask image
    bound_box = cv2.boundingRect(mask)

    if bound_box is not None:
        x, y, w, h = bound_box
        if(w < 380 and w > 50) and (h < 250 and h > 50):
            return True
        else: 
            cv2.putText(frame_display , f"(x: {x} , y: {y}) of Width: {w} and Height: {h}" , (0,600) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (0,0,255), 2 , cv2.LINE_4 ) #giving info of bodunding box
            # cv2.rectangle(frame_display, (x, y), (x + w, y + h), (255, 0, 255), 2) #making the bounding box
            return False
    else:
        return False

def fly_to(height , me):
    # fly to around 90
    while True:
        curr_height = me.get_height()
        if height - curr_height > 5:
            me.send_rc_control(0,0,10,0)
        elif height - curr_height < -5:
            me.send_rc_control(0,0,-10,0)
        else:
            print(f"curr height {me.get_height()}")
            me.send_rc_control(0,0,0,0)
            break

def instructions_localize_Height(frame_display , mask , upper_limit , lower_limit , h_upper , h_lower):
    bound_box = cv2.boundingRect(mask)
    if bound_box is not None:
        x, y, w, h = bound_box
        cv2.putText(frame_display , f"(x: {x} , y: {y}) of Width: {w} and Height: {h}" , (0,200) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
        cv2.rectangle(frame_display, (x, y), (x + w, y + h), (0, 150, 255), 2) #making the bounding box
        
        up_down = 0
        forward_back = 0
        done = False
        done1 = False
        done2 = False
        if y <= upper_limit and y >=lower_limit:
            up_down = 0
            done1 = True
        elif y > upper_limit:
            up_down = -8
        elif y < lower_limit:
            up_down = 8
        
        if h > h_upper: # might have to increase repeatedly
            forward_back = -8

        elif h < h_lower:
            forward_back = 8
        else: 
            done2 = True
            forward_back = 0

        if done1 == True and done2 == True:
            done = True
        else: 
            done = False
        cv2.putText(frame_display , f"done1: {done1} done2: {done2}" , (0,400) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
        return done , up_down , forward_back

def instructions_centralise_at( frame_display , mask , cam_center_x , cam_center_y , tolerance_x , tolerance_y , h_upper = 90 , h_lower = 70):
    bound_box = cv2.boundingRect(mask)
    if bound_box is not None:
        x, y, w, h = bound_box

        colors_center_x = (x + w)/2
        colors_center_y = (y + h)/2

        cv2.putText(frame_display , f"(x: {x} , y: {y}) of Width: {w} and Height: {h}|colors_x:{colors_center_x} , colors_y: {colors_center_y} " , (0,200) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
        cv2.circle(frame_display, (cam_center_x , cam_center_y), 2, (255, 0, 255) , -1)

        print(f"(color_x: {colors_center_x} , color_y: {colors_center_y})")
        done = False

        if colors_center_y > cam_center_y + tolerance_y:
            up_down = 8
            cv2.putText(frame_display , f"higher" , (0,400) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box

        elif colors_center_y < cam_center_y - tolerance_y:
            up_down = -8
            cv2.putText(frame_display , f"lower" , (0,400) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
        else:
            up_down = 0

        if colors_center_x > cam_center_x + tolerance_x:
            left_right = 8
            cv2.putText(frame_display , f"right" , (100,400) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box

        elif colors_center_x < cam_center_x - tolerance_x:
            left_right = -8
            cv2.putText(frame_display , f"left" , (100,400) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
        else:
            left_right = 0

        if h > h_upper: # might have to increase repeatedly
            forward_back = -8
            cv2.putText(frame_display , f"back" , (200,400) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box

        elif h < h_lower:
            forward_back = 8
            cv2.putText(frame_display , f"forward" , (200,400) , cv2.FONT_HERSHEY_COMPLEX , 0.5 , (255,0,0), 2 , cv2.LINE_4 ) #giving info of bodunding box
        else: 
            forward_back = 0
        
        if forward_back == 0 and left_right == 0:
            done = True
        
        return done , left_right, forward_back , up_down

if __name__ == "__main__":

    me = init_drone()

    # This is for initial phase of floor looking
    print("phase 1 ready")
    phase = 1 # start
    localisation_aprilTag(337)
    phase = 2 # line follower floor
    print("Phase 2 ")
    while True:
        april_id , go_xyz_x, go_xyz_y, distance = april_tags.detect(only_frames , me)
        if april_id != 294:
            FloorLineFollowerFSM(me , 6, 9)
        else:
            break
    print("cox")

    localisation_aprilTag(294)
    fly_to(70 , me)
    print(f"curr height: {me.get_height()}")
    
    phase = 3
    me.set_video_direction(me.CAMERA_FORWARD)
    # # Set the lower and upper HSV limits

    while only_frames is None: # check if all frames are ready to be read 
        continue

    while True:
        blue_mask = cv2.inRange(frame_export, blue_lower_limit, blue_upper_limit) #basically only filtering out colors of a certain lower limit hsv and upper hsv limit
        done , up_down , forward_back = instructions_localize_Height(only_frames ,blue_mask , 320 , 300 , 90 , 75)
        if done:
            print("I kindly f off")
            break
        me.send_rc_control(0 , forward_back , up_down , 0)

    while True:
        blue_mask = cv2.inRange(frame_export, blue_lower_limit, blue_upper_limit) #basically only filtering out colors of a certain lower limit hsv and upper hsv limit
        cv2.imshow("blue mask" , blue_mask)
        red_mask = cv2.inRange(frame_export, red_move_lower_limit, red_move_upper_limit)
        cv2.imshow("red mask" , red_mask)
        left_right , forward_back , up_down , yaw = blue_color_follower(only_frames , blue_mask , phase , wait_flag , me)
        me.send_rc_control(left_right , forward_back , up_down , yaw)

        if color_checker(only_frames , red_mask):
            me.send_rc_control(0,0,0,0)
            print("redtape done")
            fly_to(135 , me)
            break

    phase = 4
    fly_to(135 , me)

    while True:
        blue_mask = cv2.inRange(frame_export, blue_lower_limit, blue_upper_limit) #basically only filtering out colors of a certain lower limit hsv and upper hsv limit
        done , up_down , forward_back = instructions_localize_Height(only_frames ,blue_mask , 320 , 300 , 90 , 75)
        if done:
            print("I kindly f off")
            break
        me.send_rc_control(0 , forward_back , up_down , 0)
    
    while True:
        blue_mask = cv2.inRange(frame_export, blue_lower_limit, blue_upper_limit) #basically only filtering out colors of a certain lower limit hsv and upper hsv limit
        # cv2.imshow("blue mask" , blue_mask)
        green_mask = cv2.inRange(frame_export, green_move_lower_limit, green_move_upper_limit)
        # cv2.imshow("green mask" , green_mask)
        left_right , forward_back , up_down , yaw = blue_color_follower(only_frames , blue_mask , phase , wait_flag , me)
        me.send_rc_control(left_right , forward_back , up_down , yaw)
        if color_checker(only_frames , green_mask):
            me.send_rc_control(0,0,0,0)
            while True:
                green_mask = cv2.inRange(frame_export, green_move_lower_limit, green_move_upper_limit)
                done , left_right, forward_back , up_down = instructions_centralise_at( only_frames ,green_mask,240 , 320 , 10 , 10)
                print(f"done:{done} | lr:{left_right} | fb: {forward_back}| ud:{up_down}")
                if done:
                    me.send_rc_control(0,0,0,0)
                    print("I kindly f off from green")
                    break
                me.send_rc_control(left_right, forward_back , up_down , 0)
            fly_to(60 , me)
            print("green tape done")
            break
    print(f"curr bat: {me.get_battery()}")
    time.sleep(1)
    me.rotate_clockwise(180)
    
    me.set_video_direction(me.CAMERA_DOWNWARD)
    while only_frames is None: # check if all frames are ready to be read 
        continue
    phase = 2 # line follower floor 
    localisation_aprilTag(294)
    while True:
        april_id , go_xyz_x, go_xyz_y, distance = april_tags.detect(only_frames , me)
        if april_id != 337:
            FloorLineFollowerFSM(me , 6, 9)
        else:
            break
    phase = 1 # start
    localisation_aprilTag(337)
    me.land()

    # dongle problem plug in the blue usb. Might be best suited for USB 3.0