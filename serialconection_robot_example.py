# Robotics 23-24, Lab1 serial communications with the Scorbot example

import serial
import time
import datetime

# This function listens the serial port for wait_time seconds
# waiting for ASCII characters to be sent by the robot
# It returns the string of characters
def read_and_wait(ser, wait_time):
    output = ""
    flag = True
    start_time = time.time()
    while flag:
        # Wait until there is data waiting in the serial buffer
        if ser.in_waiting > 0:
            # Read data out of the buffer until a carriage return / new line is found
            serString = ser.readline()
            # Print the contents of the serial data
            try:
                output = serString.decode("Ascii")
                print(serString.decode("Ascii"))
            except:
                pass
        else:
            deltat = time.time() - start_time
            if deltat>wait_time:
                flag = False
    return output


def main():
    print("Starting")
    # Open the serial port COM4 to communicate with the robot (you may have to adjust
    # this instruction in case you are using a Linux OS)
    # (you must check in your computer which ports are available are, if necessary,
    # replace COM4 with the adequate COM)
    ser = serial.Serial('COM4', baudrate=9600, bytesize=8, timeout=2, parity='N', xonxoff=0)
    print("COM port in use: {0}".format(ser.name))
    
    # if 'y'== str(input('Do you want to home the robot? y/n')):
    #     ser.write(b'home\r')
    #     time.sleep(180) # homing takes a few minutes ...

    serString = "" # Used to hold data coming over UART
    ############################################################################
    # ATTENTION: Each point used was previously recorded with DEFP instruction
    
    #(from a terminal console - you can use, for example, putty or hyperterminal
    # as terminal console)
    ############################################################################
    # print('going to point P1')
    # ser.write(b'MOVE P1\r')
    # time.sleep(0.5)
    # read_and_wait(ser,2)
    # print('going to point P2')
    # ser.write(b'MOVE P2\r')
    # time.sleep(0.5)
    # read_and_wait(ser,2)

    if 'y'== str(input('DO? y/n')):
        ser.write(b'HERE pos\r')
       
        ser.write(b'LISTPV pos\r')
        a= read_and_wait(ser, 0)

    """LISTPV A

Position A

 1:-2120    2: 5743    3: 1812    4:0        5:0       

 X: 8093    Y:-1535    Z: 4225    P:-76      R:-201   """
    # closing and housekeeping
    print(a)
    ser.close()
    print('housekeeping completed - exiting')
          

if __name__ == "__main__":
    main()
