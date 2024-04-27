from picamera2 import Picamera2, MappedArray
import cv2, serial, threading, time
from config import *


front_face_detector = cv2.CascadeClassifier(front_face_path)
alt_face_detector = cv2.CascadeClassifier(alt_face_path)
profile_face_detector = cv2.CascadeClassifier(profile_face_path)

try:
    esp1 = serial.Serial("/dev/ttyACM0", 115200, timeout=0)
    esp1.reset_input_buffer()
except serial.serialutil.SerialException:
    esp1 = serial.Serial("/dev/ttyAMA10", 115200, timeout=0)
    esp1.reset_input_buffer()
except:
    print("could not find port")
    exit(1)


look_at = (0, 0)
interpolation_fact = 0.1
target_look_at = look_at

def send_coords_at_interval(interval:float):
    """
    A different thread from main will send regular updates
    """
    global look_at, esp1
    try:
        while True:
            msg_str = f'{round(look_at[0], 2)},{round(look_at[1], 2)}\n'
            data = msg_str.encode()
            num_bytes = esp1.write(data)
            print(f'[THREAD] wrote {num_bytes} bytes: {data.decode()}')
            print("unsent bytes:", esp1.out_waiting)
            time.sleep(interval)
    except Exception as e:
        print("an error occurred:",e)

def lerp(start, end, t):
    return start + t * (end - start)

def frame_update(request):
    """
    draws faces on the frame and sets the global var look_at.
    """
    global faces, w0, h0, w1, h1, motion_center, look_at, target_look_at
    # draw outines
    with MappedArray(request, "main") as m:
        largest_face = get_largest_face(faces)
        for f in faces:
            if motion_center:  
                (cx, cy) = [p * q // r for p, q, r in zip(motion_center, (w0, h0), (w1, h1))]
                target_look_at = (cx, cy)
                cv2.circle(m.array, (cx, cy), 10, (255, 0, 0), 2) 
            (x, y, w, h) = [c * n // d for c, n, d in zip(f, (w0, h0) * 2, (w1, h1) * 2)]
            if tuple(f) != largest_face:
                cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 255, 0, 0))
            else:
                cv2.rectangle(m.array, (x, y), (x + w, y + h), (255, 255, 255, 0))
                target_look_at = ((x+w)/2, (y+h)/2)
    
    look_at = target_look_at
    # (lerp(look_at[0], target_look_at[0], interpolation_fact), lerp(look_at[1], target_look_at[1], interpolation_fact) )
    

def find_faces(grey_arr):
    """
    Check the frame for all kinds of faces. If they are found return them.
    Note that this will only return one kind of face.
    """
    found_faces = front_face_detector.detectMultiScale(grey_arr, 1.1, 3)

    if len(found_faces) != 0:
        return found_faces
    
    found_faces = profile_face_detector.detectMultiScale(grey_arr, 1.1, 3)

    if len(found_faces) != 0:
        return found_faces
    
    found_faces = alt_face_detector.detectMultiScale(grey_arr, 1.1, 3)

    return found_faces

def get_largest_face(found_faces):
    max_area = 0
    largest_face = ()
    for (x,y,w,h) in found_faces:
        area = w * h
        if area > max_area:
            max_area = area
            largest_face = (x,y,w,h)	
            
    return largest_face
picam2 = Picamera2()


def init_cam():
    global w0, h0, w1, h1
    global faces, motion_center

    faces = []
    motion_center = ()
    config = picam2.create_preview_configuration(main={"size": (640, 480)},
    lores={"size": (320, 240), "format": "YUV420"})
    picam2.configure(config)
    w0, h0 = picam2.stream_configuration("main")["size"]
    w1, h1 = picam2.stream_configuration("lores")["size"]

    picam2.post_callback = frame_update

    picam2.start(show_preview=True)
    return h1


def motion_detect(gray_img_1, gray_img_2):
    motion_found = False
    biggest_area = MIN_AREA
    # Process images to see if there is motion
    differenceimage = cv2.absdiff(gray_img_1, gray_img_2)
    differenceimage = cv2.blur(differenceimage, (BLUR_SIZE,BLUR_SIZE))
    # Get threshold of difference image based on THRESHOLD_SENSITIVITY variable
    _, thresholdimage = cv2.threshold(differenceimage, THRESHOLD_SENSITIVITY, 255, cv2.THRESH_BINARY)
    # Get all the contours found in the thresholdimage
    try:
        thresholdimage, contours, _ = cv2.findContours( thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
    except:
        contours, _ = cv2.findContours( thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
    if contours != ():    # Check if Motion Found
        for c in contours:
            found_area = cv2.contourArea(c) # Get area of current contour
            if found_area > biggest_area:   # Check if it has the biggest area
                biggest_area = found_area   # If bigger then update biggest_area
                (mx, my, mw, mh) = cv2.boundingRect(c)    # get motion contour data
                motion_found = True
        if motion_found:
            motion_center = (int(mx + mw/2), int(my + mh/2))
            if verbose:
                print("motion-detect - Found Motion at px cx,cy (%i, %i) Area w%i x h%i = %i sq px" % (int(mx + mw/2), int(my + mh/2), mw, mh, biggest_area))
        else:
            motion_center = ()
    else:
        motion_center = ()
    return motion_center


def main():
    global faces, motion_center
    h1 = init_cam()

    thread = threading.Thread(target=send_coords_at_interval, args=(0.5,))
    thread.start()

    prev_frame = None
    try:
        while True:
            print("main is running")
            array = picam2.capture_array("lores")
            curr_grey_frame = array[:h1,:]
            faces = find_faces(curr_grey_frame)
            if verbose and len(faces) > 0: print("found face", faces)

            if prev_frame is not None:
                motion_center = motion_detect(prev_frame, curr_grey_frame)

            prev_frame = curr_grey_frame
    except KeyboardInterrupt:
        print("terminating program...")
    finally:
        esp1.close()
        thread.join()




if __name__ == "__main__":
    main()
