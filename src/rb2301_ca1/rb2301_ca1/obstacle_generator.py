import numpy as np
import xml.etree.ElementTree as ET
import os

height = 20
size_div = 5
width = 22
randomise = True # if false it will not change
edge_case_spacing = 0.2

workspace_directory = os.path.dirname(os.path.realpath(__file__))[:-22]
overwrite_file =  workspace_directory + '/rb2301_gz/worlds/obstacle_world_ca1.sdf'
obstacle_model = f'file:///{workspace_directory}/rb2301_gz/meshes/coke/6'

def generate_maze():
    maze_arr = np.zeros((height, width))
    maze_arr[2, int(width/2)] = 1
    for x in range(2, height-2):
        for y in range(1, width-1):
            chance = np.sum(maze_arr[x-1:x+2, y-1:y+2])
            roll = np.random.random()
            if roll < 0.5 - 0.4*2**chance + x/(3*height): # Tweak the chance of cans spawning here
                maze_arr[x,y] = 1

    maze_arr[:2, int(width/2)-1:int(width/2)+2] = 0 # Clear the starting area
    return maze_arr

def generate_edge_case(scenario):
    """Create a reproducible obstacle layout around the spawn position."""
    maze_arr = np.zeros((height, width))
    centre = int(width / 2)
    front_start = 4
    front_stop = 13
    arm_stop = 5
    arm_offset = 4

    # Preserve the original L layout, but use twice as many rows along x. With
    # the edge-case 0.2 m coordinate spacing, this keeps the same dimensions as
    # the old 0.4 m layout while closing the gaps between consecutive cans.
    maze_arr[
        front_start:front_stop,
        centre - arm_offset:centre + arm_offset + 1
    ] = 1
    if scenario == 'front_left':
        maze_arr[:arm_stop, centre + arm_offset] = 1
    elif scenario == 'front_right':
        maze_arr[:arm_stop, centre - arm_offset] = 1
    else:
        raise ValueError(
            f"Unknown obstacle scenario {scenario!r}; "
            "expected 'random', 'front_left', or 'front_right'"
        )
    return maze_arr

def add_coke_element(x, y, n):
    obstacle = ET.Element("include")
    uri = ET.Element("uri")
    uri.text = obstacle_model
    obstacle.append(uri)
    name = ET.Element("name")
    name.text = f'coke{n}'
    obstacle.append(name)
    pose = ET.Element("pose")
    pose.text = f'{x} {y} 0 0 0 0'
    obstacle.append(pose)
    return obstacle

def generate_sdf_file(scenario='random'):
    if not randomise and scenario == 'random':
        return

    if scenario == 'random':
        maze_arr = generate_maze()
    else:
        maze_arr = generate_edge_case(scenario)

    n = 1
    print(f"Generating {scenario} obstacle world...")
    tree = ET.parse(overwrite_file)
    root = tree.getroot()
    world = root[0]
    for element in reversed(world):  # Remove all coke obstacles
        if element.tag == 'include':
            world.remove(element)
    tree.write(overwrite_file)

    # Random worlds retain their original 0.4 m row spacing. The deterministic
    # L-shaped cases use 0.2 m spacing so the perpendicular arm has no
    # robot-sized openings between consecutive cans.
    x_step = 2 / size_div if scenario == 'random' else edge_case_spacing
    y_step = 1 / size_div if scenario == 'random' else edge_case_spacing

    for x in range(height):  # Add in new obstacles
        for y in range(width):
            if maze_arr[x, y] == 1:
                x_pos = x * x_step
                y_pos = (y - width / 2) * y_step
                world.append(add_coke_element(x_pos, y_pos, n))
                n += 1

    tree.write(overwrite_file)

if __name__ == '__main__':
    generate_sdf_file()
