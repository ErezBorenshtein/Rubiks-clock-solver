import serial
import time
from solve_clock_virtualy import Clock

    
def main():
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
    scramble = "UR1+ DR4- DL2- UL2- U3- R1+ D1- L3+ ALL5- y2 U2- R2+ D1+ L5- ALL0+"
    clock.scramble(scramble)
    solution = clock.solve_clock_7_simul()
    commands = clock.prepare_commands(solution)+"\n"
    #commands = "p011100 r+01000 r-10111 p001100 r+01100 r+10011 p000100 r-10001 r+11110 p010100 r+10101 r-11010 p010000 r+00100 r+01011 p110000 r+01100 r+00011 p110100 r-11101 r+00010\n"
    print(commands)
    ser = serial.Serial("COM4",9600)
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

    while True:
        try:
            data = ser.readline().decode().strip()
            print(data)
        finally:
            time.sleep(0.01)

if __name__ == "__main__":
    main()
