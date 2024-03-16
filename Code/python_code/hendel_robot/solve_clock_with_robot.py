import serial
import time
from solve_clock_virtualy import Clock

    
def main():
    clock = Clock()
    clock.set_clock([
                    #front
                    1,1,1,
                    1,1,1,
                    0,0,0,
                    #back
                    11,0,11,
                    0,0,0,
                    0,0,0])
    clock.print_clock()
    commands = clock.solve_clock_7_simul()
    solution = clock.prepare_commands(commands)
    print(solution)
    ser = serial.Serial("COM4",9600)
    time.sleep(3)
    ser.write(solution.encode())
    #ser.write("p010100".encode())
    while True:
        print(ser.readline().decode())
        time.sleep(0.01)

main()


