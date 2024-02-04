import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # If frame is read correctly ret is True
    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break
    
    lower_red = np.array([160,100,100])
    upper_red = np.array([179,255,255])
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_red,upper_red)    
    result = cv2.bitwise_and(frame,frame,mask = mask)
    frame = result
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image = frame
    target_x = 0
    target_y = 0
    # Iterate over the contours
    for i in contours:
        # Get the bounding rectangle of the contour
        approx = cv2.approxPolyDP(i, 0.04 * cv2.arcLength(i, True), True)
        if len(approx) == 4 and cv2.contourArea(approx) > 1000:
        # Draw bounding rectangle around the contour
            # x, y, w, h = cv2.boundingRect(approx)
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)   
             M = cv2.moments(i)
             if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.drawContours(image, [i], -1, (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 7, (0, 255, 0), -1)
                cv2.putText(image, "center", (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image, 'coor: '+str(cx-20)+','+str(cy-20), (cx-20,cy-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))   
                target_x = cx-20
                target_y = cy-20   
    frame = image
    height, width = frame.shape[:2]
    # print("height is ",height)
    # print("height is ",width)
    x, y, w, h = 0, 0, 640, 360
    l = 200
    color = (0, 255, 0)  # Rectangle color (green)
    thickness = 2  # Rectangle line thickness
    # centre =
    q1 = [640,0,1280,360]
    q2 = [0,0,640,360]
    q3 = [0,360,640,720]
    q4 = [640,360,1280,720]
    q5 = [w-l,h-l,w+l,h+l]
   
    sector = [q1,q2,q3,q4,q5]
    for q in sector:
        cv2.rectangle(frame, (q[0],q[1]),(q[2],q[3]), color)
    if(q5[0] < target_x and target_x < q5[2] and q5[1] < target_y and target_y < q5[3]):
        print("descend")
    elif(q1[0] < target_x and target_x < q1[2] and q1[1] < target_y and target_y < q1[3]):
        print("move down and right")    
    elif(q2[0] < target_x and target_x < q2[2] and q2[1] < target_y and target_y < q2[3]):
        print("move down and left")    
    elif(q3[0] < target_x and target_x < q3[2] and q3[1] < target_y and target_y < q3[3]):
        print("move up and left")    
    elif(q4[0] < target_x and target_x < q4[2] and q4[1] < target_y and target_y < q4[3]):
        print("move up and right")
    
    # Draw a rectangle on the frame
    # cv2.rectangle(frame, (x, y), (x+w, y+h), color, thickness)
    # cv2.rectangle(frame, (640, 360), (1280, 0), color, thickness)
    # cv2.rectangle(frame, (0, 720), (640, 360), color, thickness)
    # cv2.rectangle(frame, (640, 360), (1280, 720), color, thickness)
    # Display the resulting frame
    cv2.imshow('Webcam - Press Q to quit', frame)
    
    # Break the loop when 'Q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
