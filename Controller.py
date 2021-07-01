from time import sleep
import serial
import struct

ser = serial.Serial("COM5", 250000)

speed = 0.5e-3
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
    data = [1, 170, 35.45, 142.6, 160, 71.5, 150, speed, acceleration, 0, 0, 0, -1]
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

#goHome()
goHome()

for num in range(1):
    #sendMovement1(250, 36 + 90, 120, 142.5, 180)
    sendMovement1(170, 150, 60 + 180, 142.5, 180)
    sendMovement1(80, 50, 180, 142.5, 180)
    sendMovement1(80, 110, 90, 142.5, 180)
    
goHome()
"""
#sendMovement1(180, 130, 45, 180, 180)

sendMovement1(300, 130, 45, 105, 225)
sendMovement1(135, 130, 45, 225, 145)
sendMovement1(180, 130, 45, 180, 180)

sendMovement1(180, 130, 225, 180, 180)

sendMovement1(300, 130, 225, 225, 225)
sendMovement1(135, 130, 225, 105, 145)
sendMovement1(180, 130, 225, 180, 180)

sendMovement1(225, 45, 90, 135, 180)
sendMovement1(135, 130, 45, 180, 180)
"""





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
