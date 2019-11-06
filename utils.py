def print_matrix(matrix):
    print("[")
    for i in matrix:
        print("[", end="")
        for j in i:
            print(f"{j:2f}\t", end="")
        print("]")
    print("]")

def print_array(arr):
    print("[")
    for i in arr:
        print(f"{i},")
    print("]")
