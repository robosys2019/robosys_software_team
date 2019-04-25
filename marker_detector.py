import cv2
import cv2.aruco as aruco
import numpy as np

class MarkerDetector:
    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()
    
    def detect_markers(self, frame):
        # Detecting markers in real time
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markers, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

        if (len(markers) == 0):
            pass
        else:
            frame = aruco.drawDetectedMarkers(frame, markers)
            print("[MarkerDetector] Found marker in image.")
        
        if frame != []:
            cv2.imshow("RGB Image Window", frame)
            cv2.waitKey(3)
        
        # # Using webcam
        # cap = cv2.VideoCapture(1)
        # while(True):
        #     ret, frame = cap.read() #640x480

        #     # Detecting markers in real time
        #     gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #     markers, ids, _ = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

        #     if (len(markers) == 0):
        #         # print("No marker found.")
        #         pass
        #     else:
        #         aruco.drawDetectedMarkers(frame, markers)
        #         print("[MarkerDetector] Found marker in image. Id: " + str(ids[0]))
        #         corners = np.reshape(markers, (-1, 2))
        #         for c in corners:
        #             print(c)

    def test(self):
        # Using webcam
        cap = cv2.VideoCapture(0)
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
                print("[MarkerDetector] Found marker in image. Id: " + str(ids[0]))
                corners = np.reshape(markers, (-1, 2))
                for c in corners:
                    print(c)

            # Show image, press q to exit
            cv2.imshow('clean image', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Clean everything up
        cv2.destroyAllWindows()

if __name__ == '__main__':
    md = MarkerDetector()
    md.test()
