front_face_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
alt_face_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt2.xml'
profile_face_path = '/usr/share/opencv4/haarcascades/haarcascade_profileface.xml'

# Display Settings
debug = True        # Set to False for no data display
verbose = False      # Add extra detailed information
show_fps = True     # show frames per second processing speed
window_on = True    # Set to True displays opencv windows (GUI desktop reqd)
diff_window_on = False  # Show OpenCV image difference window
thresh_window_on = False  # Show OpenCV image Threshold window
CIRCLE_SIZE = 8     # diameter of circle to show motion location in window
LINE_THICKNESS = 2  # thickness of bounding line in pixels
WINDOW_BIGGER = 1   # Resize multiplier for OpenCV Status Window
                    # if window_on=True then makes opencv window bigger
                    # Note if the window is larger than 1 then a reduced frame rate will occur

                   
# Camera Settings
# ---------------

# FPS counter
FRAME_COUNTER = 1000  # Used for display of FPS (frames/second)

timer_motion = 3      # seconds delay after no motion before looking for face
timer_face = 2        # seconds delay after no face found before starting pan search
timer_pan = 1         # seconds delay between pan search repositioning movements

# OpenCV Motion Tracking Settings
MIN_AREA = 2000       # sq pixels - exclude all motion contours less than or equal to this Area
THRESHOLD_SENSITIVITY = 25
BLUR_SIZE = 10
