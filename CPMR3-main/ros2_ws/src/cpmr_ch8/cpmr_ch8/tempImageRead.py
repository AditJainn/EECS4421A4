# # Define a list of tuples
# my_list = [(1, 'apple'), (2, 'banana'), (3, 'orange'), (4, 'grape')]

# # Define the tuple you want to find
# target_tuple = (3, 'orange')

# try:
#     # Get the index of the target tuple
#     index = my_list.index(target_tuple)
    
#     # Print the index
#     print(f"Index of {target_tuple}: {index}")
# except ValueError:
#     print(f"{target_tuple} not found in the list.")
points = [
    [1.3, 1.5], [1.48, 1.45], [1.65, 1.37], [1.8, 1.53], [1.78, 1.79],
    [2.0, 1.77], [2.13, 1.73], [2.27, 1.64], [2.37, 1.7], [2.42, 1.55],
    [2.61, 1.57], [2.66, 1.64], [2.83, 1.68], [2.89, 1.84], [2.79, 1.94],
    [2.96, 2.04], [3.08, 2.1], [3.15, 2.02], [3.14, 1.94], [3.18, 1.83],
    [3.22, 1.79], [3.17, 1.64], [3.09, 1.43], [3.04, 1.24], [3.23, 1.22],
    [3.27, 1.0], [3.24, 0.96], [3.19, 0.93], [3.06, 0.93], [3.0, 0.85],
    [2.9, 0.64], [2.91, 0.6], [2.83, 0.39], [2.73, 0.26], [2.64, 0.19],
    [2.6, 0.23], [2.52, 0.43], [2.57, 0.53], [2.5, 0.5]]

x = points[0]
x.append(0.3)

print(x) 