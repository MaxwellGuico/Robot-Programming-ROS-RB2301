import json
import os
import numpy as np
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

height = 20
size_div = 5# original is 5
width = 22
randomise = False # if false it will not change
edge_case_spacing = 0.2

source_directory = Path(__file__).resolve().parents[2]
workspace_directory = source_directory.parent
overwrite_file = source_directory / 'rb2301_gz/worlds/obstacle_world_ca1.sdf'
obstacle_model = (source_directory / 'rb2301_gz/meshes/coke/6').as_uri()
output_root = workspace_directory / 'ca1_runs'
latest_run_file = output_root / 'latest_run.txt'

def generate_maze():
    # np.random.seed(int(os.environ.get('RB2301_OBSTACLE_SEED', '1')))
    # np.random.seed(5)
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

def create_run_directory():
    """Create a unique folder for this generated obstacle environment."""
    output_root.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_directory = output_root / timestamp
    run_directory.mkdir()
    latest_run_file.write_text(str(run_directory.resolve()), encoding="utf-8")
    return run_directory

def save_can_layout(maze_arr, x_step, y_step, scenario, run_directory):
    save_environment_metadata(maze_arr, x_step, y_step, scenario, run_directory)
    """Save a top-down JPEG of the exact can layout written to Gazebo."""
    import matplotlib
    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    can_rows, can_columns = np.nonzero(maze_arr)
    can_x = can_rows * x_step
    can_y = (can_columns - width / 2) * y_step

    fig, ax = plt.subplots(figsize=(8, 10))
    ax.scatter(
        can_y,
        can_x,
        s=70,
        color="firebrick",
        edgecolors="black",
        linewidths=0.4,
        label="Cans",
        zorder=2,
    )
    ax.scatter(
        0.0,
        0.0,
        marker="^",
        s=140,
        color="royalblue",
        edgecolors="black",
        label="Robot start",
        zorder=3,
    )

    padding = max(x_step, y_step)
    ax.set_xlim(width / 2 * y_step + padding, -width / 2 * y_step - padding)
    ax.set_ylim(-padding, height * x_step + padding)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"Generated can layout ({scenario})")
    ax.set_xlabel("y (m, +left)")
    ax.set_ylabel("x (m, +forward)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(run_directory / "can_layout.jpg", format="jpeg", dpi=150)
    plt.close(fig)



def save_environment_metadata(maze_arr, x_step, y_step, scenario, run_directory):
    """Save the generated can positions and evaluation finish line."""
    can_rows, can_columns = np.nonzero(maze_arr)
    can_positions = [
        {
            "x": round(float(row * x_step), 10),
            "y": round(float((column - width / 2) * y_step), 10),
        }
        for row, column in zip(can_rows, can_columns)
    ]
    metadata = {
        "scenario": scenario,
        "number_of_cans": len(can_positions),
        "last_can_row_x": max(position["x"] for position in can_positions),
        "coordinate_convention": "+x forward/up, +y left",
        "can_positions": can_positions,
    }
    metadata_path = run_directory / "environment.json"
    metadata_path.write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )

def generate_sdf_file(scenario='front_left'):
    
    if randomise and scenario == 'random':
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
    x_step = 2 /5 if scenario == 'random' else edge_case_spacing
    y_step = 1 / size_div if scenario == 'random' else edge_case_spacing

    for x in range(height):  # Add in new obstacles
        for y in range(width):
            if maze_arr[x, y] == 1:
                x_pos = x * x_step
                y_pos = (y - width / 2) * y_step
                world.append(add_coke_element(x_pos, y_pos, n))
                n += 1

    tree.write(overwrite_file)
    run_directory = create_run_directory()
    save_can_layout(maze_arr, x_step, y_step, scenario, run_directory)
    save_environment_metadata(maze_arr, x_step, y_step, scenario, run_directory)
    print(f"Saved environment images in {run_directory}")
    return run_directory

if __name__ == '__main__':
    generate_sdf_file()
