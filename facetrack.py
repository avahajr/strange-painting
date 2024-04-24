from picamera2 import Picamera2, MappedArray
import cv2
import serial
from config import *

front_face_detector = cv2.CascadeClassifier(front_face_path)
alt_face_detector = cv2.CascadeClassifier(alt_face_path)
profile_face_detector = cv2.CascadeClassifier(profile_face_path)

esp1 = serial.Serial("/dev/ttyACM0", 115200)

def frame_updates(request):
    global faces, w0, h0, w1, h1, motion_center

    # draw outines
    with MappedArray(request, "main") as m:
        largest_face = get_largest_face(faces)
        for f in faces:
            (x, y, w, h) = [c * n // d for c, n, d in zip(f, (w0, h0) * 2, (w1, h1) * 2)]
            if tuple(f) != largest_face:
                cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 255, 0, 0))
            else:
                cv2.rectangle(m.array, (x, y), (x + w, y + h), (255, 255, 255, 0))

        if motion_center:  
            (cx, cy) = [p * q // r for p, q, r in zip(motion_center, (w0, h0), (w1, h1))]
            cv2.circle(m.array, (cx, cy), 10, (255, 0, 0), 2) 


    esp1.write(f'helloworld\n'.encode('utf-8'))
    line = esp1.readline().decode('utf-8').rstrip()
    print(line)

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

    picam2.post_callback = frame_updates

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
    prev_frame = None

    while True:
        array = picam2.capture_array("lores")
        curr_grey_frame = array[:h1,:]
        faces = find_faces(curr_grey_frame)
        if verbose and len(faces) > 0: print("found face", faces)

        if prev_frame is not None:
            motion_center = motion_detect(prev_frame, curr_grey_frame)

        prev_frame = curr_grey_frame




if __name__ == "__main__":
    main()
