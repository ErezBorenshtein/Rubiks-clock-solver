import pandas as pd
from solve_clock_virtualy import Clock

def main():
    clock = Clock()
    
    # Create an empty list to store dictionaries
    data = []

    for i in range(10000):
        scramble = clock.generate_scramble()
        clock.scramble(scramble)
        state = clock.get_state()
        state2 = state.copy()
        commands = clock.solve_clock_7_simul()
        solution = clock.prepare_commands(commands)
        
        # Append a dictionary to the list without 'Scramble'
        data.append({'State': state2, 'Solution': solution})

    # Create a DataFrame from the list of dictionaries
    df = pd.DataFrame(data)

    # Write the DataFrame to a CSV file
    df.to_csv('clock_solutions.csv', index=False)

if __name__ == "__main__":
    main()