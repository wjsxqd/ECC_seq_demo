import random
from collections import deque

random.seed(114514)

# Random sequence generator
def seq_generator(demo_len):
    seq = []
    base = ["A", "G", "C", "T"]
    for _ in range(demo_len):
        cur_base = random.choice(base)
        seq.append(cur_base)
    print (f"generated sequence: {''.join(seq)}")
    return seq

# Convert ECC_seq to a 3D grid
def convert_to_3d(ECC_seq, n):
    grid = [[[0 for _ in range(n)] for _ in range(n)] for _ in range(n)]
    for i in range(n):
        for j in range(n):
            for k in range(n):
                if ECC_seq[0][i] + ECC_seq[1][j] + ECC_seq[2][k] == 1 or ECC_seq[0][i] + ECC_seq[1][j] + ECC_seq[2][k] == 3:
                    grid[i][j][k] = 1
    return grid

def is_valid(x, y, z, n, grid):
    """Check if a point is valid and passable within the grid."""
    return 0 <= x < n and 0 <= y < n and 0 <= z < n and grid[x][y][z] == 1

def find_all_paths(n, grid, penalty_1=1, penalty_2=2, max_cost=5): ## you can change the penalties for insertion, deletion and threshold for max cost
    """Find all possible paths from (0,0,0) to any (i, j, k) where any of i, j, k == n-1."""
    # Directions for the three types of moves
    directions_type1 = [(1, 0, 1), (1, 1, 0), (0, 1, 1)]
    directions_type2 = [(2, 1, 1), (1, 2, 1), (1, 1, 2),
                        (2, 1, 2), (2, 2, 1), (1, 2, 2), 
                        (3, 1, 1), (1, 3, 1), (1, 1, 3)]
    directions_type3 = [(1, 1, 1)]
    
    # Initialize the queue with the starting point and the initial cost (0, 0, 0, current_cost)
    queue = deque([(0, 0, 0, 0, [])])
    all_paths = []
    
    while queue:
        x, y, z, current_cost, path = queue.popleft()
        path.append((x, y, z))
        
        # If any coordinate reaches n-1, save the path
        if x == n-1 or y == n-1 or z == n-1:
            all_paths.append((x, y, z, current_cost, list(path)))
            continue
        
        # If the cost exceeds the max_cost, terminate this path
        if current_cost > max_cost:
            continue
        
        # Type 3 moves first
        for dx, dy, dz in directions_type3:
            nx, ny, nz = x + dx, y + dy, z + dz
            if is_valid(nx, ny, nz, n, grid):
                queue.append((nx, ny, nz, current_cost, list(path)))
                # If a type 3 move is valid, continue to the next iteration of the while loop
                break
        else:
            # If no type 3 move was valid, try type 1 and type 2 moves
            # Type 1 moves
            for dx, dy, dz in directions_type1:
                nx, ny, nz = x + dx, y + dy, z + dz
                if is_valid(nx, ny, nz, n, grid):
                    queue.append((nx, ny, nz, current_cost + penalty_1, list(path)))
            
            # Type 2 moves
            for dx, dy, dz in directions_type2:
                nx, ny, nz = x + dx, y + dy, z + dz
                if is_valid(nx, ny, nz, n, grid):
                    queue.append((nx, ny, nz, current_cost + penalty_2, list(path)))
    
    return all_paths

# Decode paths back to the original sequence
def decode_path_to_seq(path, ECC_seq):
    decoded_seq = []
    for x, y, z in path:
        if ECC_seq[0][x] == 1 and ECC_seq[1][y] == 1 and ECC_seq[2][z] == 1:
            decoded_seq.append('A')
        elif ECC_seq[0][x] == 1 and ECC_seq[1][y] == 0 and ECC_seq[2][z] == 0:
            decoded_seq.append('C')
        elif ECC_seq[0][x] == 0 and ECC_seq[1][y] == 1 and ECC_seq[2][z] == 0:
            decoded_seq.append('G')
        elif ECC_seq[0][x] == 0 and ECC_seq[1][y] == 0 and ECC_seq[2][z] == 1:
            decoded_seq.append('T')
    return decoded_seq


# Main code
seq = seq_generator(demo_len = 200) ## you can use a real sequence, or a randomly generated sequence

# Convert the sequence to ECC sequence
ECC_seq = [[], [], []]
for cur_base in seq:
    if cur_base == "A":
        ECC_seq[0].append(1)
        ECC_seq[1].append(1)
        ECC_seq[2].append(1)
    elif cur_base == "C":
        ECC_seq[0].append(1)
        ECC_seq[1].append(0)
        ECC_seq[2].append(0)
    elif cur_base == "G":
        ECC_seq[0].append(0)
        ECC_seq[1].append(1)
        ECC_seq[2].append(0)
    elif cur_base == "T":
        ECC_seq[0].append(0)
        ECC_seq[1].append(0)
        ECC_seq[2].append(1)

# To test its robustness, we perform random insertion or deletion in 3 ecc sequences.
ECC_seq[0].insert(37, 1)
ECC_seq[1].insert(98, 0)
ECC_seq[2].pop(52)

# Find the minimum length of the three ECC sequences. This is the maximal length for decoding.
n = min(len(ECC_seq[0]),len(ECC_seq[1]), len(ECC_seq[2]))
grid = convert_to_3d(ECC_seq, n)

# Find all paths in the generated grid
paths = find_all_paths(n, grid)

paths = sorted(paths, key=lambda x: x[3])
# Print the found paths and their decoded sequences
for path_info in paths:
    x, y, z, cost, path = path_info
    decoded_seq = ''.join(decode_path_to_seq(path, ECC_seq))
    print(f"Cost: {cost}, Decoded sequence: {decoded_seq}")
    if decoded_seq in ''.join(seq):
        print("This is the original sequence.")
        break