from time import sleep
import serial
import struct

ser = serial.Serial("COM18", 9600)

speed = 0.8e-3
acceleration = 1.0e-10

print("Starting...")
sleep(0.5)
print("Connected!")
#dataRecived = ser.read_until(b"\n")
#arduino_input = dataRecived.decode("ascii")

#while "Initialization COMPLETE" not in arduino_input:
#    dataRecived = ser.read_until(b"\n")
#    print("Teensy: " + dataRecived.decode("ascii")[:-2])
#    arduino_input = dataRecived.decode("ascii")


def sendMovement1(axis1, axis2, axis3, axis4, axis6):
    data = [1, axis1, axis2, axis3, axis4, 0, axis6, speed, acceleration, 0, 0, 0, -1]
    print(len(data))
    print(data)
    for i in range(len(data)):
        ser.write(struct.pack("d", float(data[i])))
        print("\nWriting " + str(data[i]) + " to Teensy")
        dataRecived = ser.read_until(b"\n")
        print("Teensy: " + dataRecived.decode("ascii")[:-2])
        dataRecived = ser.read_until(b"\n")
        print("Teensy: " + dataRecived.decode("ascii")[:-2])
    while ser.inWaiting() > 0:
            dataRecived = ser.read_until(b"\n")
            print("Teensy: " + dataRecived.decode("ascii")[:-2])
    print("Done!")

def goHome():
    data = [1, 0, 0, 0, 180, 0, 0, speed, acceleration, 0, 0, 0, -1]
    print(len(data))
    print(data)
    for i in range(len(data)):
        ser.write(struct.pack("d", float(data[i])))
        print("\nWriting " + str(data[i]) + " to Teensy")
        dataRecived = ser.read_until(b"\n")
        print("Teensy: " + dataRecived.decode("ascii")[:-2])
        dataRecived = ser.read_until(b"\n")
        print("Teensy: " + dataRecived.decode("ascii")[:-2])
    while ser.inWaiting() > 0:
            dataRecived = ser.read_until(b"\n")
            print("Teensy: " + dataRecived.decode("ascii")[:-2])
    print("Done!")

goHome()
sendMovement1(180, 0, 180, 180, 180)
sendMovement1(180, 130, 45, 180, 180)

sendMovement1(300, 130, 45, 105, 225)
sendMovement1(135, 130, 45, 225, 145)
sendMovement1(180, 130, 45, 180, 180)

sendMovement1(180, 130, 225, 180, 180)

sendMovement1(300, 130, 225, 225, 225)
sendMovement1(135, 130, 225, 105, 145)
sendMovement1(180, 130, 225, 180, 180)

sendMovement1(225, 45, 90, 135, 180)
sendMovement1(135, 130, 45, 180, 180)
goHome()





"""
data = ""
while data != "quit":

    data = input("Command: ")
    if (data == "test"):
        while ser.inWaiting() > 0:
            dataRecived = ser.read_until(b"\n")
            print("Teensy: " + dataRecived.decode("ascii")[:-2])

        sendMovement1()
    elif (data != "quit"):
        print(struct.pack("d", float(data)))
        ser.write(struct.pack("d", float(data)))
        dataRecived = ser.read_until(b"\n")
        print("Teensy: " + dataRecived.decode("ascii")[:-2])
        dataRecived = ser.read_until(b"\n")
        print("Teensy: " + dataRecived.decode("ascii")[:-2])
"""
print("Goodbye!")
ser.close()
