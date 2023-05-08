import cv2
  
  
# define a video capture object
vid = cv2.VideoCapture(1)

vid2 = cv2.VideoCapture(2)
  
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    ret2, frame2 = vid2.read()

    # Display the resulting frame
    if ret:
        cv2.imshow('frame', frame)
    if ret2:
        cv2.imshow('frame2', frame2)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()