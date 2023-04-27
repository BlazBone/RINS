from typing import List, Tuple
def read_path_log_orientation(filename: str) -> List[Tuple[float, float, float, float, float]]:
    """
    Reads a path from a log file. (file must be generated with /amcl_pose)

    Args:
        filename (str): path to the log file
    Return:
        path (list): list of tuples (x, y, z, z_orientation, w_orientation)
    """
    path = []
    with open(filename, "r") as f:
        for line in f:
            
            if line.strip().startswith("position:"):
                
                x = float(f.readline().strip().split(" ")[1])
                y = float(f.readline().strip().split(" ")[1])
                z = float(f.readline().strip().split(" ")[1])
                # skippings text "orinetation: "
                f.readline()
                x_orientation = float(f.readline().strip().split(" ")[1])
                y_orientation = float(f.readline().strip().split(" ")[1])
                z_orientation = float(f.readline().strip().split(" ")[1])
                w_orientation = float(f.readline().strip().split(" ")[1])

                path.append((x, y, z, z_orientation, w_orientation))
    return path

def read_path_log(filename: str) -> List[Tuple[float, float, float]]:
    """
    Reads a path from a log file. (file must be generated with /clicked_point) 

    Is deprected, use read_path_log_orientation instead

    Args:
        filename (str): path to the log file
    Return:
        path (list): list of tuples (x, y, z, z_orientation, w_orientation)
    """
    path = []
    with open(filename, "r") as f:
        for line in f:
            if line.strip().startswith("x: "):
                x = float(line.strip().split(" ")[1])
                y = float(f.readline().strip().split(" ")[1])
                z = float(f.readline().strip().split(" ")[1])
                path.append((x, y, z))
    return path