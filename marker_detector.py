import cv2
import cv2.aruco as aruco
import numpy as np

class MarkerDetector:
    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()
    
    def detect_markers(self, frame):
        # Using webcam
        cap = cv2.VideoCapture(1)
        while(True):
            ret, frame = cap.read() #640x480

            # Detecting markers in real time
            gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            markers, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

            if (len(markers) == 0):
                # print("No marker found.")
                pass
            else:
                aruco.drawDetectedMarkers(frame, markers)
                # print("[MarkerDetector] Found marker in image. Ids: " + str(ids))
                corners = np.reshape(markers, (-1, 2))
                ns = ""
                we = ""
                
                if corners[0][1] > corners[1][1] and corners[0][0] < corners[2][0]:
                    ns = "North"
                elif corners[0][1] < corners[1][1] and corners[0][0] > corners[2][0]:
                    ns = "South"

                if corners[0][1] > corners[2][1] and corners[0][1] > corners[3][1] and corners[0][0] > corners[1][0] and corners[0][0] > corners[2][0]:
                    we = "West"
                elif corners[0][1] < corners[2][1] and corners[0][1] < corners[3][1] and corners[0][0] < corners[1][0] and corners[0][0] < corners[2][0]:
                    we = "East"

                print("[MarkerDetector] Rover is facing: " + ns + " " + we)

            # Show image, press q to exit
            cv2.imshow('Webcam Image', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Clean everything up
        cv2.destroyAllWindows()

if __name__ == '__main__':
    md = MarkerDetector()
    md.detect_markers()
