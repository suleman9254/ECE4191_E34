import cv2
import time
from ultralytics import YOLO
import numpy as np

import sys
print(sys.executable)


model = YOLO(r'C:\Users\robin\Downloads\ECE4191_E34\RobitShit\best.pt')

img = cv2.VideoCapture(0)

#camera calibration from calibration
camera_matrix = np.array([
    [649.47113436, 0.00000000, 249.59894886],
    [0.00000000, 650.00394065, 252.46785209],
    [0.00000000, 0.00000000, 1.00000000]
])

dist_coeffs = np.array([[-0.12860389, -0.3649167, 0.00291756, -0.00061939, 0.32511248]])

#need to measure tennis ball this was the range
actual_diameter_cm = (6.54 + 6.86) / 2  

#Confidence threshold
confidence_threshold = 0.7  

#CHAT GPT poggers 
while True:
    ret, frame = img.read()

    if not ret:
        print("Failed to grab frame")
        break

    #Undistort
    frame = cv2.flip(frame,-1)
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    #for measuring latency can remove
    start_time = time.time()

    #Run YOLO inference
    results = model(frame)

    #for latency can remove
    end_time = time.time()

    #Calculate latency
    latency = end_time - start_time
    print(f"Latency: {latency:.4f} seconds")

    #Extract bounding box information
    annotated_frame = frame.copy()

    #Iterate thorugh results
    for result in results:
        boxes = result.boxes
        for box in boxes:
            #Bounding box coords
            xyxy = box.xyxy[0].tolist()  
            
            #Confidence score
            confidence = box.conf[0].item()  

            #confidence threshold 
            if confidence >= confidence_threshold:
                x1, y1, x2, y2 = xyxy

                #diameter of pixal + center of object
                perceived_diameter_px = max(x2 - x1, y2 - y1)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                #estimate distance + pos
                if perceived_diameter_px > 0:
                    x_position = (actual_diameter_cm * camera_matrix[0, 0]) / perceived_diameter_px
                    print(f"Estimated Distance: {x_position:.2f} cm")
                    
                    y_position = (center_x - camera_matrix[0, 2]) * x_position / camera_matrix[0, 0]
                    height  = -(center_y - camera_matrix[1, 2]) * x_position / camera_matrix[1, 1]
                
                    print(f"Object Position (x, y, z): ({x_position:.2f} cm, {y_position:.2f} cm, {height:.2f} cm)")
                   
                    #annocate to frame can delete. 
                    annotated_frame = cv2.putText(
                        annotated_frame,
                        f"(x, y, z): ({x_position:.2f} cm, {y_position:.2f} cm, {height:.2f} cm)",
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 100, 0),
                        2,
                        cv2.LINE_AA
                    )

                    #bounding box can delete 
                    annotated_frame = cv2.rectangle(
                        annotated_frame,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (255, 100, 0),
                        2
                    )
                    #can delete confidence 
                    annotated_frame = cv2.putText(
                        annotated_frame,
                        f"Conf: {confidence:.2f}",
                        (int(x1), int(y1) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 100, 0),
                        2,
                        cv2.LINE_AA
                    )

    #display annocations 
    # cv2.imshow("YOLOv8 Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

img.release()
cv2.destroyAllWindows()