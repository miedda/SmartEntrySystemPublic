#! /usr/bin/python

# This code has been developed with reference to the openCV facial_recognition
# library developed by Caronline Dunn. It is available here: https://github.com/carolinedunn/facial_recognition

# It requires building open-CV. Good instructions for the raspberry Pi are here:
# https://core-electronics.com.au/guides/face-identify-raspberry-pi/?gclid=CjwKCAjwy_aUBhACEiwA2IHHQHh6_g7R9qUF_AzduzoL1I3Oa3MXOpzs6GbP2kZF4IKGhYINA_72ahoCHHoQAvD_BwE

from curses import BUTTON2_TRIPLE_CLICKED
from cv2 import FONT_HERSHEY_COMPLEX
from imutils.video import VideoStream
from imutils import paths
import face_recognition
import imutils
import pickle
import time
import cv2
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from picamera import PiCamera
from picamera.array import PiRGBArray
import os

# CONSTANTS
BROKER_ADDRESS = "10.1.1.9"
BUTTON_PIN = 27

# GLOBALS
buttonFlag = False

#  Callbacks
def buttonCallback(*args):
    global buttonFlag
    buttonFlag = True

# This function compares the face encoding of anyone in frame to the registered
# encodings. It returns true if a registered user is found.
def checkIdentity(currentFace, encodedUsers):
    # Get the next frame, resize it to known format and find the faces. Then
    # encode the faces and store them in an array
    frame = video.read()
    frame = imutils.resize(frame, width = 500)
    boxes = face_recognition.face_locations(frame)
    encodings = face_recognition.face_encodings(frame, boxes)
    faces = []

    # Iterate through each encoded face in the frame, and match them with
    # the registered faces.
    for encoding in encodings:
        matches = face_recognition.compare_faces(encodedUsers["encodings"], encoding)
        face = "unknown"

        # Make a list of the true matches, then count each match. The match
        # with the most counts is the person recognised.
        if True in matches:
            # Get the match indexes
            indexes = [i for (i, b) in enumerate(matches) if b]
            counts = {}
            
            # Iterate through the registered faces, and update the count for
            # each one.
            for i in indexes:
                face = encodedUsers["names"][i]
                counts[face] = counts.get(face, 0) + 1

            # Get the face with the most counts, and set it to the current face
            face = max(counts, key=counts.get)

            if currentFace != face:
                currentFace = face
                print(currentFace)

        # If the face is known add the face to the array of faces in the image.
        if face != "unknown":
            faces.append(face)

    # Check if a registered face is seen and set the authorisedFace variable
    if len(faces) > 0:
        return True
    else:
        return False

# This function registers a new user. It can only be entered on program start
# as a bug in the imUtil library used prevents the camera being released properly.
# The function first captures photos of the person, and then encodes the images
# and any other images in the dataset directory as authorised users.
def registerUser(encodingFile):
    camera = PiCamera()
    camera.resolution = (512, 304)
    camera.framerate = 10
    rawImage = PiRGBArray(camera, size=(512, 304))

    # Create file structure to store data
    datasetPath = "/home/pi/control/dataset/"
    id = len(os.listdir(datasetPath)) + 1
    idDir = str(id)
    if not os.path.isdir(datasetPath):
        print("Creating directory")
        os.mkdir(datasetPath)
    os.mkdir(datasetPath + idDir + "/")

    # Capture photos on button press. Stop when 10 photos have been taken.
    counter = 0
    for frame in camera.capture_continuous(rawImage, format="bgr", use_video_port=True):
        image = frame.array
        cv2.imshow("Register user", image)
        rawImage.truncate(0)

        cv2.waitKey(1)
        rawImage.truncate(0)
        global buttonFlag
        if buttonFlag:
            buttonFlag = False
            img_name = datasetPath + idDir + "/image_{}.jpg".format(counter)
            cv2.imwrite(img_name, image)
            print("{} written!".format(img_name))
            counter += 1
        elif counter >= 10:
            print("capture complete")
            break

    # Encode the photos in the dataset folder so they can be used for facial
    # recognition.
    print("processing face")
    imagePaths = list(paths.list_images(datasetPath))
    print(imagePaths)

    knownEncodings = []
    knownNames = []

    for (i, imagePath) in enumerate(imagePaths):
        print("processing {}/{}".format(i + 1, len(imagePaths)))
        name = imagePath.split(os.path.sep)[-2]

        image = cv2.imread(imagePath)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        boxes = face_recognition.face_locations(rgb, model="hog")
        encodings = face_recognition.face_encodings(rgb, boxes)

        for encoding in encodings:
            knownEncodings.append(encoding)
            knownNames.append(name)

    # Write the encoded values to disk for later use.
    data = {"encodings": knownEncodings, "names": knownNames}
    f = open(encodingFile, "wb")
    f.write(pickle.dumps(data))
    f.close()

    print("Encoding complete")
    os.system('systemctl reboot -i')

# Main
try:
    # Set up program variables
    # Set up buttons
    print("Set up GPIO")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(BUTTON_PIN, edge=GPIO.FALLING, callback=buttonCallback, bouncetime=200)

    # Set up MQTT connection
    print("Connect MQTT")
    client = mqtt.Client(client_id="control")
    client.connect(BROKER_ADDRESS)

    # Initalise variables
    encodingFile = "/home/pi/control/encodings.pickle"
    # Load registered faces
    encodedUsers = pickle.loads(open(encodingFile, "rb").read())

    # Check to see if user is trying to load images
    if not GPIO.input(BUTTON_PIN):
        registerUser(encodingFile)

    # Set up video stream
    print("Start video stream")
    video = VideoStream(usePiCamera=True).start()
    time.sleep(2)
    currentFace = "unknown"
    authorisedFace = False

    # Loop and examine frames of the video for known faces. If a known face is
    # found then add it to the list of currently recognised faces.
    while True:
        authorisedFace = checkIdentity(currentFace, encodedUsers)

        # If a button has been pressed, and there is a registered user present
        # send an unlock request
        if buttonFlag:
            buttonFlag = False
            if authorisedFace:
                client.publish("Unlock", "")
                print("Unlock request sent")
                authorisedFace = False
            else:
                print("Not registered user")

except KeyboardInterrupt:
    GPIO.cleanup()
    cv2.destroyAllWindows()
    video.stop()
    quit()