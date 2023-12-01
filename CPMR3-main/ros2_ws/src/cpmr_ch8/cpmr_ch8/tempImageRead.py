# Define a list of tuples
my_list = [(1, 'apple'), (2, 'banana'), (3, 'orange'), (4, 'grape')]

# Define the tuple you want to find
target_tuple = (3, 'orange')

try:
    # Get the index of the target tuple
    index = my_list.index(target_tuple)
    
    # Print the index
    print(f"Index of {target_tuple}: {index}")
except ValueError:
    print(f"{target_tuple} not found in the list.")
