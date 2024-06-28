import numpy as np
from scipy.stats import mode
from collections import Counter

class BufferManager:
    def __init__(self, num_rows, num_cols):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.buffer = [[0 for _ in range(num_cols)] for _ in range(num_rows)]
        self.pointer = 0
    def prepare_positions(self,num_rows, num_cols):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.pointer = 0
        self.buffer = [[[0,0] for _ in range(num_cols)] for _ in range(num_rows)]
    def add_to_buffer(self, array):
        if len(array) != self.num_cols:
            raise ValueError(f"Array must contain exactly {self.num_cols} integers.")
        
        row = self.pointer
        self.buffer[row] = array
        self.pointer = (self.pointer + 1) % self.num_rows
    
    def print_buffer(self):
        for row in self.buffer:
            print(row)
    
    def get_pointer(self):
        return self.pointer
    
    def get_most_popular_numbers(self):
        num_columns = len(self.buffer[0])
        most_popular = []
    
        for col in range(num_columns):
            column_data = [row[col] for row in self.buffer]
            most_common = Counter(column_data).most_common(1)[0][0]
            most_popular.append(most_common)
    
        return most_popular
    
    def average_positions(self):

        num_columns = len(self.buffer[0])
        averages = []
        
        for col in range(num_columns):
            filtered_points = [row[col] for row in self.buffer if row[col] != [0, 0]]
            if not filtered_points:
                averages.append((0, 0))
                continue
            sum_x = sum(point[0] for point in filtered_points)
            sum_y = sum(point[1] for point in filtered_points)
            avg_x = sum_x / len(filtered_points)
            avg_y = sum_y / len(filtered_points)
            averages.append((avg_x, avg_y))
        
        return averages

# Example usage:
num_rows = 9
num_cols = 20
buffer_manager = BufferManager(num_rows, num_cols)
new_array = list(range(1, num_cols + 1))

# Add the new array to the buffer multiple times for testing
for _ in range(num_rows):
    buffer_manager.add_to_buffer(new_array)

# Print the buffer to verify the update
buffer_manager.print_buffer()

# Get the most popular numbers in each column
most_popular_numbers = buffer_manager.get_most_popular_numbers()
print("Most popular numbers in each column:", most_popular_numbers)
