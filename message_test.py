import math

def make_message(angle=0, distance=0, size=6):
        # TODO: Check this format
        angle_deg = int(angle* 180 / math.pi)
        if(angle_deg >= 256):
                angle_msg = [angle_deg-255, 255]
        else:
                angle_msg = [0,angle_deg]

        distance_msg = int(distance*12)

        message = bytearray([angle_msg[0], angle_msg[1], distance_msg])
        #print(len(bytearray))
        # ^ this probably needs to change based on the format of angle and distance

        # print(message)
        # self.ser.write(message)
        #return message

make_message(6.02139, 3.8)
