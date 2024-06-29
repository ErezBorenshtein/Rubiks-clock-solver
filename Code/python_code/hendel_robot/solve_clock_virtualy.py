import numpy as np
class Clock:
    def __init__(self):
        self.clock_pins = [0, 0, 0, 0] # 0 for down and 1 for up 
        self.clock = [0] * 18 # all the clocks are in the 12 o'clock position
        self.PUR, self.PDR, self.PDL, self.PUL = 0, 1, 2, 3 # pins numbers
        self.UL, self.U, self.UR, self.L, self.C, self.R, self.DL, self.D, self.DR = 0, 1, 2, 3, 4, 5, 6, 7, 8 # wheel numbers
        self.FRONT, self.BACK = 0, 9 # for the rotate function
        
        self.num_to_wheel = {
            2: "UR",
            8: "DR",
            6: "DL",
            0: "UL",
            1: "U",
            5: "R",
            7: "D",
            3: "L",
            4: "C"
        }

        self.wheel_to_num = {
            "UR": 2,
            "DR": 8,
            "DL": 6,
            "UL": 0,
            "U": 1,
            "R": 5,
            "D": 7,
            "L": 3,
            "C": 4
        }

        self.pin_mapping = {
            #wheel to pin
            "UR": (self.PUR, None),
            "DR": (self.PDR, None),
            "DL": (self.PDL, None),
            "UL": (self.PUL, None),
            "R": (self.PUR, self.PDR),
            "D": (self.PDR, self.PDL),
            "L": (self.PDL, self.PUL),
            "U": (self.PUR, self.PUL),
            "ALL": (self.PUR, self.PDR, self.PDL, self.PUL)
        }

        self.pin_num_mapping = {
            #for the front side
            self.UR: (self.PUR, None),
            self.DR: (self.PDR, None),
            self.DL: (self.PDL, None),
            self.UL: (self.PUL, None),
            self.R: (self.PUR, self.PDR),
            self.D: (self.PDR, self.PDL),
            self.L: (self.PDL, self.PUL),
            self.U: (self.PUR, self.PUL),
            #for the back side
            self.UR+9:(self.PUL, None),
            self.DR+9:(self.PDL, None),
            self.DL+9:(self.PDR, None),
            self.UL+9:(self.PUR, None),
        }

        self.wheel_mapping_str = {
            #Name to number
            "UR": self.UR,
            "DR": self.DR,
            "DL": self.DL,
            "UL": self.UL,
            "R": self.R,
            "D": self.D,
            "L": self.L,
            "U": self.U,
        }

        self.pin_mapping_reversed = {
            #pin to wheel(numbers)
            self.PUR: self.UR,
            self.PDR: self.DR,
            self.PDL: self.DL,
            self.PUL: self.UL
        }

        self.wheels_around_pin = {
            #wheels around the pin
            self.PUR: (self.UR, self.U, self.R, self.C),
            self.PDR: (self.DR, self.D, self.C, self.R),
            self.PDL: (self.DL, self.L, self.C, self.D),
            self.PUL: (self.UL, self.U, self.C, self.L)
        }


        self.wheel_mapping_rotated = {
            #clocks after y2 rotation
            "UR": "UL",
            "DR": "DL",
            "DL": "DR",
            "UL": "UR",
            "R": "L",
            "L": "R",
            "D": "U",
            "U": "D",
            "ALL": "ALL",
            "C":"C"
        }

        self.pin_side_to_wheel = {
            #what wheel to rotate for each set of pins
            "U": "UR",
            "R": "UR",
            "D": "DR",
            "L": "DL",
            "UR": "UR",
            "DR": "DR",
            "DL": "DL",
            "UL": "UL",
            "ALL": "UR",
            "ALL_BUT_0": "UL",
            "ALL_BUT_1": "UR",
            "ALL_BUT_2": "DR",
            "ALL_BUT_3": "DL",
            "UR_AND_DL": "UR",
            "DL_AND_UR": "UR",
            "UL_AND_DR": "UL",
            "DR_AND_UL": "UL",
        }

        self.pin_side_to_wheel_after_rotation = {
            #what wheel to rotate for each set of pins after y2 rotation
            "U": "DR",
            "R": "UR",
            "D": "UR",
            "L": "DL",
            "ALL": "UR",
            "UL": "UR",
            "UR": "UL",
            "DR": "DL",
            "DL": "DR",
        }

    def set_clock(self, clock):
        #set the clock to a specific configuration(if the state is valid)
        for i in [(0,11),(2,9),(6,17),(8,15)]:
            if clock[i[0]] != (12-clock[i[1]])%12:
                raise Exception("Invalid clock configuration, clock corners in both sides must be the same")
            else:
                self.clock = clock

    def reset(self):
        # Reset the clock to its initial state
        self.clock_pins = [0, 0, 0, 0]
        self.clock = [0] * 18

    def is_solved(self):
        # Check if the clock is in a solved state
        return self.clock == [0] * 18

    def flip_pins(self):
        # Flip the pins after y2 rotaqtion
        return [1 if pin == 0 else 0 for pin in reversed(self.clock_pins)]

    def print_clock(self):
        for i in range(len(self.clock)):
            print(self.clock[i], end=" ")
            if (i + 1) % 3 == 0 and (i + 1) % 9 != 0:
                print("\n", end="")
            elif (i + 1) % 9 == 0 and i + 1 != len(self.clock):
                print("\nBack\n", end="")
        print("\n")

    def rotate_wheel(self, wheel, num_of_rot, is_rotated):
        #simulates the rotation of the wheel on the clock

        #print("\nRotated", wheel, num_of_rot, "times","is_rotated=",is_rotated,"\n")
    
        if is_rotated == True:
            wheel =  self.pin_side_to_wheel_after_rotation[wheel] #filp the wheel if the move is after a y2 rotation
        else:
            wheel = self.pin_side_to_wheel[wheel]

        #rotates the wheel from the front side and returns the wheels that were rotated
        wheels_side_1 = self.rotate_internal(wheel, num_of_rot, self.FRONT, self.clock_pins) 

        #rotates the wheel from the back side and returns the wheels that were rotated
        wheels_side_2 =self.rotate_internal(self.wheel_mapping_rotated[wheel], -num_of_rot, self.BACK, self.flip_pins())

        #add the wheels that were rotated to the final movement
        if len(wheels_side_1) == 0: #for each move just one side is rotated, if this side is the back we'll conv
            wheels_side_1 = wheels_side_2
            wheels_side_2 = []
            i=0
            for wheel in wheels_side_1:
                if wheel in [0,2,6,8,9,11,15,17]: #the wheels that are in the corners of the clock
                    wheels_side_2.append(wheel)
            wheels_side_1 = wheels_side_2
            i=0     
            for wheel in wheels_side_1:
                wheels_side_1[i] =self.pin_mapping_reversed[self.pin_num_mapping[wheel][0]] #convert the wheels to the pins
                i+=1

        final_movement = []
        for i in range(len(wheels_side_1)):
            if len(self.num_to_wheel[wheels_side_1[i]]) != 1: #if the wheel is a corner wheel
                final_movement.append(self.num_to_wheel[wheels_side_1[i]]) #add the wheel to the final movement

        if num_of_rot>6: # you can get top any position by rotating the wheel maximum 6 times(clockwise or counterclockwise)
            num_of_rot = -(12%num_of_rot)
        if num_of_rot<-6:
            num_of_rot = num_of_rot+12

        final_movement_str = f"r:{'+' if num_of_rot >= 0 else ''}{num_of_rot}"

        for i in range(len(final_movement)):
            final_movement_str+=final_movement[i]+","

        print(final_movement_str)
        #self.print_clock()

        return final_movement_str
        

    def rotate_internal(self, wheel, num_of_rot, offset, pins):
        #rotates the wheel for onece for each side(front or back) and returns the wheels that were rotated

        final_movement =[]
        if len(wheel) == 2:
            a=0
            # Check if the pin connected to the wheel is up
            if pins[self.pin_num_mapping[self.wheel_to_num[wheel]][0]] == 1:
            # Iterate through all the pins
                for option in pins:
                    # Check if the pin is up
                    if option == 1:
                        # Iterate through the wheels around the pin and add them to the final movement
                        for wheel_around_part in self.wheels_around_pin[a]:
                            final_movement.append(wheel_around_part+offset)
                    a+=1
                # Remove duplicates from the final movement
                final_movement = list(set(final_movement))

                # Rotate the clocks in the final movement by the specified number of rotations
                for move in final_movement:
                    self.clock[move]+=num_of_rot
                    self.clock[move] = self.clock[move]%12

            else:
                # If the wheel connected to the pin is not up
                for option in range(4):
                    if pins[option] == 0:
                        self.clock[self.pin_mapping_reversed[option]+offset] += num_of_rot
                        self.clock[self.pin_mapping_reversed[option]+offset] = self.clock[self.pin_mapping_reversed[option]+offset]%12
        return final_movement

    def scramble_to_rotation(self, scramble):

        # converts the scramble to a list of moves that can be used to rotate the clock
        #TODO: make the function more efficient

        enhanced_scramble=[]
        listed_scramble = scramble.split(" ")
        for move in listed_scramble:
            if move[0:3].isalpha() and len(move) ==5:
                if move[4] == "+":
                    enhanced_scramble.append([move[0:3],int(move[3])])
                else:
                    enhanced_scramble.append([move[0:3],-(int(move[3]))])

            elif move[0:2].isalpha() ==True:
                if len(move) ==2:
                    enhanced_scramble.append(move)
                else:
                    if move[3] == "+":
                        enhanced_scramble.append([move[0:2],int(move[2])])
                    else: 
                        enhanced_scramble.append([move[0:2],-(int(move[2]))])

            elif move[0].isalpha() == True and move[0] != "y":
                if move[2] == "+":
                    enhanced_scramble.append([move[0],int(move[1])])
                else:
                    enhanced_scramble.append([move[0],-(int(move[1]))])
            elif move[0] == "y":
                enhanced_scramble.append([move[0],move[1]])

        return enhanced_scramble

    def set_pins(self,move,is_rotated):
        # set the pins

        for pin in range(4): #reset the pins
            self.clock_pins[pin] = 0

        if move == "ALL": #all pins need to be up
            for pin in range(4):
                self.clock_pins[pin] = 1

        elif "BUT" in move: #all pins need to be up except one
            pin_not_moving = int(move[8])
            for pin in range(4):
                if pin != pin_not_moving:
                    self.clock_pins[pin] = 1

        elif "AND" in move: #two pins(in diagonal) need to be up
            self.clock_pins[self.pin_mapping[move[0:2]][0]] = 1
            self.clock_pins[self.pin_mapping[move[7:9]][0]] = 1

        elif len(move) == 1: #one pin needs to be up
            for pin in self.pin_mapping[move]:
                self.clock_pins[pin] = 1

        else:
            self.clock_pins[self.pin_num_mapping[self.wheel_mapping_str[move]][0]] = 1

        #ret_val = f"p{self.clock_pins[0]}{self.clock_pins[1]}{self.clock_pins[2]}{self.clock_pins[3]}"

        if is_rotated == True: #if the move is after a y2 rotation the pins need to be flipped
            flipped_pins = self.flip_pins()
            for pin in range(4):
                self.clock_pins[pin] = flipped_pins[pin]
                
        ret_val = f"p{self.clock_pins[0]}{self.clock_pins[1]}{self.clock_pins[2]}{self.clock_pins[3]}"
        print(ret_val)
        return ret_val

    def count_clocks_in_same_time(self):

        #* this function will be used for the reinforcement learning model to calculate the reward
        #TODO: fix bugs in the function
        i=1
        used_clocks = []
        for i in range (9):
            if i != 4:
                current = self.clock[i]
                if i not in used_clocks:
                    for similar_clock in self.wheels_around_pin[self.pin_mapping[self.num_to_wheel[i]][0]]:
                        if self.clock[similar_clock] == current and similar_clock != i:
                            if i not in used_clocks:
                                    used_clocks.append(i)
                            if similar_clock not in used_clocks:
                                used_clocks.append(similar_clock)
                        used_clocks.sort()
            i+=1
        
        for i in range(9,18):
            if i !=13:
                current = self.clock[i]
                for similar_clock in self.wheels_around_pin[self.pin_mapping[self.wheel_mapping_rotated[self.num_to_wheel[i-9]]][0]]:
                    similar_clock+=9
                    if self.clock[similar_clock] == current and similar_clock != i:
                    #if self.clock[similar_clock] == self.clock[current] and similar_clock != i:
                        if i not in used_clocks:
                                    used_clocks.append(i)
                        if similar_clock not in used_clocks:
                            
                            used_clocks.append(similar_clock)
                        used_clocks.sort()
            i+=1
        return len(used_clocks)

        
            

    def scramble(self,scramble):
        #function that scrambels the clock virtualy

        scramble = self.scramble_to_rotation(scramble)
        is_rotated = False
        for move in scramble:
            if move[0] == "y":
                is_rotated = True

            elif type(move) == str:
                self.set_pins(move,is_rotated)

            elif is_rotated:
                self.set_pins(move[0],is_rotated)
                self.rotate_wheel(self.wheel_mapping_rotated[move[0]],-(move[1]),is_rotated)

            elif not is_rotated:
                self.set_pins(move[0],is_rotated)
                self.rotate_wheel(move[0],move[1],is_rotated)

            elif move[0] == "ALL":
                self.set_pins(move[0],is_rotated)
                self.rotate_wheel(move[0],move[1],is_rotated)
            

    def solve_clock_7_simul(self):

        # this function will be used to solve the clock virtually using the 7 simul method
        # the 7 simul method is a common and efficient way to solve the Rubik'c clock
        # the function returns a list of commands that can be used to solve the clock with the robots

        solution = []
        commands = []
        solution.append([(self.clock[13]-self.clock[16])%12,(self.clock[17]-self.clock[14]+self.clock[7]-self.clock[5])%12]) #First and second moves
        solution.append([(self.clock[5]-self.clock[7])%12,(self.clock[16]-self.clock[14])%12]) # third and fourth moves
        solution.append([(self.clock[6]-self.clock[7]+self.clock[4]-self.clock[1]+self.clock[12]-self.clock[9]+self.clock[14])%12, 
                               ((self.clock[10]-self.clock[13]+self.clock[16])+(self.clock[8]-self.clock[5])+(self.clock[0]-self.clock[3]))%12]) #fifth adn sixth moves

        #First 2 simul moves
        commands.append(self.set_pins("UL",True))
        
        commands.append(self.rotate_wheel("UL",solution[0][0],True))
        commands.append(self.rotate_wheel("L",solution[0][1],True))

        #Second 2 simul moves
        commands.append(self.set_pins("L",True))
        commands.append(self.rotate_wheel("R",solution[1][1],True))
        commands.append(self.rotate_wheel("L",solution[1][0],True))

        #Third 2 simul moves
        commands.append(self.set_pins("UL",False))
        commands.append(self.rotate_wheel("UL",self.clock[5]-self.clock[4],False))
        commands.append(self.rotate_wheel("L",-(self.clock[8]-self.clock[4]),True))

        #Fourth 2 simul moves
        commands.append(self.set_pins("UL_AND_DR",False))
        commands.append(self.rotate_wheel("DR",solution[2][0],False))
        commands.append(self.rotate_wheel("DL",solution[2][1],False))

        #Fifth 2 simul moves
        commands.append(self.set_pins("DR",False))
        commands.append(self.rotate_wheel("DR",-(self.clock[4]-self.clock[1]),False))
        commands.append(self.rotate_wheel("UR",-(self.clock[2]-self.clock[1]),False))

        #Sixth 2 simul moves
        commands.append(self.set_pins("R",False))
        commands.append(self.rotate_wheel("R",-(self.clock[1]-self.clock[3]),False))
        commands.append(self.rotate_wheel("UL",(self.clock[1]-self.clock[0]),False))

        #Seventh 2 simul moves
        commands.append(self.set_pins("ALL_BUT_2",False))
        commands.append(self.rotate_wheel("R",12-self.clock[0],False))
        commands.append(self.rotate_wheel("DL",12-self.clock[6],False))

        #print(commands)
        return commands

    def generate_scramble(self):
        # this function is used to generate a random scramble for the clock NOT! acording to the WCA regulations

        symbols = ["+", "-"]
        scramble_structure = [
            "UR", "DR", "DL", "UL", "U", "R", "D", "L", "ALL",
            "y", "U", "R", "D", "L", "ALL",
            "UR", "DL", "UL"
        ]
        scramble = ""
        for move in scramble_structure:
            symbol = np.random.choice(symbols)
            number = np.random.randint(1, 7)  # Random number between 1 and 6
            scramble += f"{move}{number}{symbol} "

        
        scramble = scramble[:-1]

        return scramble
        
    def prepare_command(self,command):
        if command[0] == "r":
            new_command = command[command.find(":")+3:]
            list_of_commands = new_command.split(",")
            final_command = f"r{command[2:4]}"
            if "UR" in list_of_commands:
                final_command+="1"
            else:
                final_command+="0"

            if "DR" in list_of_commands:
                final_command+="1"
            else:
                final_command+="0"

            if "DL" in list_of_commands:
                final_command+="1"
            else:
                final_command+="0"

            if "UL" in list_of_commands:
                final_command+="1"
            else:
                final_command+="0"
        else:
            final_command = command+"00"
            #final_command = command+"  "
        return final_command
    
    def prepare_commands(self,commands):
        final_commands = ""
        for command in commands:
            final_commands+=self.prepare_command(command)
            final_commands+=" "
        return final_commands[:-1]
    
    def get_state(self):
        return self.clock
    
    def optimize_commnds_7_simul(self,commands):

        #!this code works just if the commands are already prepared and only if the solution was generated with the solve_clock_7_simul method
        
        optimized_commands = []
        sliced_commands = []
        splited_commands = commands.split(" ")
        for i in range(0, len(splited_commands), 3):
            sublist = splited_commands[i:i+3]  # Get the next 3 elements
            sliced_commands.append(sublist)
        
        for chank in sliced_commands:
            optimized_commands.append(chank[0]+"00")
            rotate = "r"+ ''.join(chank[1][1:3] if wheel == "1" else "_" for wheel in chank[1][3:])

            rotate = rotate.replace("_",chank[2][1:3])
            
            optimized_commands.append(rotate)
        
        return " ".join(optimized_commands)



def main():
    clock = Clock()
    #scramble = "UR1+ DR4- DL2- UL2- U3- R1+ D1- L3+ ALL5- y2 U2- R2+ D1+ L5- ALL0+"
    #clock.scramble(scramble)
    #clock_instance.print_clock()
    #commands =clock.solve_clock_7_simul()
    #commands = clock.prepare_commands(commands)
    #commands = "p011100 r-21000 r-20111 p001100 r-31100 r-30011 p000100 r+40001 r-31110 p010100 r+10101 r+61010 p010000 r-50100 r-41011 p110000 r-51100 r+10011 p110100 r-21101 r+10010"
    #new_commands = clock.optimize_commnds_7_simul(commands)
    #print(commands)
    #scramble = "UR1+ DR4- DL2- UL2- U3- R1+ D1- L3+ ALL5- y2 U2- R2+ D1+ L5- ALL0+"
    #clock.scramble(scramble)
    #clock.set_clock([
    #                #front
    #                2,3,2,
    #                3,4,3,
    #                2,3,2,
    #                #back
    #                10,0,10,
    #                0,0,0,
    #                10,0,10])
    #commands = clock.solve_clock_7_simul()
    #clock.print_clock()
    #scramble = "UR1+ DR4- DL2- UL2- U3- R1+ D1- L3+ ALL5- y2 U2- R2+ D1+ L5- ALL0+"
    #scramble = "UR1- DR5- DL5- UL3- U3+ R3- D1- L6+ ALL2- y2 U1- R2+ D3+ L0+ ALL2+"
    scramble = "UR4- DR2- DL1+ UL5+ U1- R4+ D1- L6+ ALL1+ y2 U3- R2+ D2+ L3- ALL3+"
    clock.scramble(scramble)
    clock.print_clock()
    print("--------------------------------------")
    commands = clock.solve_clock_7_simul()
    print(commands)
    new_commands = clock.prepare_commands(commands)
    print(new_commands)
    #new_commands = "p011100 r-21000 r-20111 p001100 r-31100 r-30011 p000100 r+40001 r-31110 p010100 r+10101 r+61010 p010000 r-50100 r-41011 p110000 r-51100 r+10011 p110100 r-21101 r+10010"
    #print(new_commands)
    print(clock.optimize_commnds_7_simul(new_commands))
    
if __name__ == "__main__":
    main()
