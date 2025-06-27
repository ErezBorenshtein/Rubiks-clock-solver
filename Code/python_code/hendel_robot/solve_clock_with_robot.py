import serial
import time
from solve_clock_virtualy import Clock
from recognize_clock  import * 
# import winsound
import threading
#import keyboard

BLACK = (0,0,0)
WHITE = (255,255,255)


def beep():
    frequency = 1000  # Frequency of the beep sound in Hertz
    duration = 500    # Duration of the beep in milliseconds (500 ms = 0.5 seconds)
    # winsound.Beep(frequency, duration)

def beep_in_thread():
    beep_thread = threading.Thread(target=beep)
    beep_thread.start()

    
def solve_without_camera():
    clock = Clock()
    #clock.set_clock([
    #                #front
    #                0,4,5,
    #                3,11,8,
    #                10,7,2,
    #                #back
    #                7,11,0,
    #                1,2,5,
    #                10,7,2])
    #clock.print_clock()
    scramble = "UR0+ DR2- DL3+ UL4- U2+ R5- D4+ L1- ALL3+ y2 U1+ R3- D0+ L1+ ALL1+"
    clock.scramble(scramble)
    solution = clock.solve_clock_7_simul()
    commands = clock.prepare_commands(solution)+"\n"
    commands = clock.optimize_commnds_7_simul(commands)+" \n"
    #commands = "p011100 r+01000 r-10111 p001100 r+01100 r+10011 p000100 r-10001 r+11110 p010100 r+10101 r-11010 p010000 r+00100 r+01011 p110000 r+01100 r+00011 p110100 r-11101 r+00010\n"
    print(commands)
    ser = serial.Serial("COM4",115200)
    #time.sleep(3)
    x = ser.readline().decode().strip()
    print(x)
    while(x != "ready"):
        x = ser.readline().decode().strip()
        print(x)

    
    print("Arduino is ready to receive data.")
    ser.write(commands.encode())
    ser.flush()
    print("Data sent over serial successfully.")

    ser.write("start\n".encode())

    while True:
        try:
            data = ser.readline().decode().strip() 
            print(data)
        finally:
            time.sleep(0.01)




def solve_with_camera():

    ser = None
    try:
        ser = serial.Serial("COM4",115200, timeout=1)   #windows
    except:
        try:
            ser = serial.Serial("/dev/ttyUSB0",115200, timeout=1)   #linux
        except:
            ser = serial.Serial("/dev/ttyUSB1",115200, timeout=1)   #linux
            
    print("Solving clock with camera")
    centers_buffer.prepare_positions(20,9)

    camera = cv2.VideoCapture(0)
    
    hour1 = read_clock(camera,WHITE)
    print("hour1: ",hour1)
    hour2 = read_clock(camera,BLACK)
    print("hour2: ",hour2)

    print("Clock read successfully")
    cv2.destroyAllWindows()

    clock_state = hour1 + hour2
    clock = Clock()

    if(not clock.set_clock(clock_state)):
        print("Clock state is not valid")
    
    clock.print()

    commands = clock.solve_clock_7_simul()
    commands = clock.prepare_commands(commands)
    commands = clock.optimize_commnds_7_simul(commands)+" \n"
    print("commands (PC):" + commands)
    
                 
    #commands = "'p01110000 r-3+3+3+3 p00110000 r+5+5-1-1 p00010000 r-2-2-2+0 p01010000 r+0-5+0-5 p01000000 r+6+2+6+6 p11000000 r-1-1-1-1 p11010000 r+6+6-4+6 \n'"
    
    #commands = "p01110000 r-5-2-2-2 p00110000 r-5-5+3+3 p00010000 r-2-2-2-1 p01010000 p10100000 p01010000\n"
    # commands = "p01110000 r-2-4-4-4 p00010000 r-2-2-2-1 p01010000 r-1-1-1-1 p11000000 r+3+3+0+0 p11010000 r+3+3+5+3\n"

    time.sleep(1)
    # commands = "p01110000 r+1+2+2+2 p00010000 r+4+4+4+2 p01010000 r+1+1+1+1 p01000000 r-3-5-3-3 p11000000 r+0+0-3-3"
    # commands = "p00010000 r+4+4+4+2"
    # commands = "p00000000"
    data = ""

    while(data != "ready"):
        data = ser.readline().decode().strip()
        if data != "":
            print(data)

    print("Arduino is ready to receive data.")
    # ser.write(commands.encode())

    commands = commands.strip("'")

    for part in commands.split(" "):  # Split on space
        ser.write((part + " ").encode())  # Add the space back
        ser.flush()
        time.sleep(0.02)  # 10ms delay between chunks

    ser.write(b'\n')  # Final newline to trigger Arduino reading


    ser.flush()
    print("Data sent over serial successfully.")

    space_was_pressed = False
    guiness_challnge = False
    if guiness_challnge:
        while True:   
            if keyboard.is_pressed("space") and not space_was_pressed:

                beep_in_thread()
                
                ser.write("start\n".encode())
                print("Start signal sent to arduino")
                #break
                space_was_pressed = True

            try:
                if ser.in_waiting > 0:
                    data = ser.readline().decode().strip()
                    print(data)
                    if data == "done":
                        beep_in_thread()
                        break
            finally:
                time.sleep(0.001)
    else:
        ser.write("start\n".encode())
        ser.flush()
        print("Start signal sent to arduino")
        while True: 
            try:
                data = ser.readline().decode().strip()
                if data =="" or data == " ":
                    continue
                print(data)
            finally:
                time.sleep(0.01)



if __name__ == "__main__":
    solve_with_camera()
 