import freenect
import cv2
import numpy as np
import numpy.ma as ma
import imutils
from imutils.object_detection import non_max_suppression
from imutils import paths

import scipy.misc

back_clipping = 200
tilt_degs = 20


# Scrollhandler for back_clipping
def change_back_clipping(value):
    global back_clipping
    back_clipping = value


# Scrollhandler for tilt_degs
# def change_tilt_degs(value):
#     tilt_degs = value - 20
#     freenect.set_tilt_degs(tilt_degs)

# mdev = freenect.open_device(freenect.init(), 0)
# freenect.set_depth_mode(0,freenect.RESOLUTION_MEDIUM, freenect.DEPTH_MM)
# freenect.set_depth_mode(mdev, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_REGISTERED)
# freenect.runloop(dev=mdev)


depth_window_name = 'Depth Frame'
cv2.namedWindow(depth_window_name)
# cv2.createTrackbar('Background Clip Value', depth_window_name, back_clipping, 0xFF, change_back_clipping)
# cv2.createTrackbar('Tilt Degrees', depth_window_name, tilt_degs, 40, change_tilt_degs)

rbg_window_name = 'RBG Frame'
cv2.namedWindow(rbg_window_name)


rbgchange_window_name = 'RBG Change Frame'
cv2.namedWindow(rbgchange_window_name)


# function to get RGB image from kinect
def get_video():
    global mdev
    array, _ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


# function to get depth image from kinect
def get_depth():
    global mdev
    array, _ = freenect.sync_get_depth(format =  freenect.DEPTH_REGISTERED)
    array = array.astype(np.int32)
    return array


def draw_detections(img, detectionResults, thickness=1, resizeFactor = 1):
    for rects in detectionResults:
        for x, y, w, h in rects:
            # the HOG detector returns slightly larger rectangles than the real objects.
            # so we slightly shrink the rectangles to get a nicer output.
            x = int(x * resizeFactor)
            y = int(y * resizeFactor)
            h = int(h * resizeFactor)
            w = int(w * resizeFactor)
            pad_w, pad_h = int(0.15 * w), int(0.05 * h)
            cv2.rectangle(img, (x + pad_w, y + pad_h), (x + w - pad_w, y + h - pad_h), (0, 255, 0), thickness)





def clip_and_prepare_frame(data):
    global back_clipping
    # use globals in context
    # shift elements for correct usage
    data >>= 1
    # convert to uint8 values (unsigned int with 1 byte)
    data = data.astype(np.uint8)
    # !! MAGIC - clip data behind and before the object
    np.clip(data, 0, back_clipping, data)
    data -= back_clipping
    return data



face_cascade = cv2.CascadeClassifier('./haar/haarcascade_frontalface_default.xml')
faceprofile_cascade = cv2.CascadeClassifier('./haar/haarcascade_profileface.xml')
upperbody_cascade = cv2.CascadeClassifier('./haar/haarcascade_upperbody.xml')
fullbody_cascade = cv2.CascadeClassifier('./haar/haarcascade_fullbody.xml')
resizefactor = 0.5
ROI = [200, 195, 100,75]
DEPTHCAMERA_TO_RGBCAMERA_OFFSET = -45 #Pixels
background_subtractor = cv2.BackgroundSubtractorMOG2(history=10, varThreshold=13)
learningRate = -1.0
savedFrameNumber = 0
amount_of_movement_detected_threshold = 1000;
def saveFrame():
    global savedFrameNumber

    depthimage = scipy.misc.toimage(cropped_depth, high=np.max(cropped_depth), low=np.min(cropped_depth), mode='I')
    cv2.imwrite('../captured/frame' + str(savedFrameNumber) + '.png', cropped_frame)
    depthimage.save('../captured/depth' + str(savedFrameNumber) + '.png')
    savedFrameNumber = savedFrameNumber + 1

if __name__ == "__main__":
    while 1:
        # get a frame from RGB camera
        frame = get_video()
        # print("Pre-processing frame.")
        frame = cv2.GaussianBlur(frame,(5,5),0)
        # Convert the image to grayscale.
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # scale input image for faster processing
        frame = cv2.resize(frame, (0, 0), fx=resizefactor, fy=resizefactor)
        # frame = cv2.equalizeHist(frame)
        face_results = face_cascade.detectMultiScale(frame, 1.3,10)
        faceprofile_results = faceprofile_cascade.detectMultiScale(frame, 1.3,10)
        upperbody_results = upperbody_cascade.detectMultiScale(frame, 1.3,10)
        fullbody_results = fullbody_cascade.detectMultiScale(frame, 1.3,10)
        foundHuman = [face_results,faceprofile_results,upperbody_results,fullbody_results]

        frame = cv2.resize(frame, (0, 0), fx=(1+resizefactor), fy=(1+resizefactor))


        if len(foundHuman)>0:
            # print("Found a hooman")
            draw_detections(frame, foundHuman,1,resizefactor+1)

        # get a frame from depth sensor
        depth = get_depth()
        # depth = clip_and_prepare_frame(depth)
        # depth = cv2.resize(depth, (frame.shape[1], frame.shape[0]))

        cropped_frame = frame[ ROI[1]:ROI[1]+ROI[3],ROI[0]:ROI[0]+ROI[2]]
        cropped_depth = depth[ ROI[1]:ROI[1]+ROI[3],ROI[0]+DEPTHCAMERA_TO_RGBCAMERA_OFFSET:ROI[0]+ROI[2]+DEPTHCAMERA_TO_RGBCAMERA_OFFSET]
        fgMask = background_subtractor.apply(cropped_frame, None, learningRate)
        amount_of_movement_detected = len(cropped_frame[fgMask>0])
        if fgMask.any():
            cv2.imshow(rbgchange_window_name, fgMask)
            print("Movement detected: "+str(amount_of_movement_detected))
        if amount_of_movement_detected>amount_of_movement_detected_threshold:
            print("Saving frame number "+str(savedFrameNumber)+" max pix:"+str(np.amax(cropped_depth)))
            saveFrame()


        cv2.rectangle(frame, (ROI[0], ROI[1]), (ROI[0] + ROI[2], ROI[1] + ROI[3]), (255, 255, 0), 2)
        # cv2.rectangle(depth, (ROI[0] + DEPTHCAMERA_TO_RGBCAMERA_OFFSET, ROI[1]), (ROI[0] + ROI[2]+DEPTHCAMERA_TO_RGBCAMERA_OFFSET, ROI[1] + ROI[3]), (255, 255, 0), 2)
        # display RGB image
        cv2.imshow(rbg_window_name, cropped_frame)
        # display depth image
        cv2.imshow(depth_window_name, depth)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:  # wait for ESC key to exit
            cv2.destroyAllWindows()
        elif k == ord('s'):  # wait for 's' key to save and exit
            saveFrame()
    cv2.destroyAllWindows()
