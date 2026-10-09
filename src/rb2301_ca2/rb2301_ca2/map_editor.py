import numpy as np
import matplotlib.pyplot as plt

filename = "ca2_sim_map.npy"

grid = np.load(filename)
original_grid = grid.copy()

fig, ax = plt.subplots(figsize=(12, 9))

img = ax.imshow(
    grid.T,
    origin="lower",
    cmap="gray_r",
    vmin=0,
    vmax=99,
    interpolation="nearest"
)

ax.set_title(
    "Left Drag: Add Wall (99) | Right Drag: Remove Wall (0) | S: Save | R: Reset"
)

ax.set_xlabel("Grid X")
ax.set_ylabel("Grid Y")

# Display cell boundaries
ax.set_xticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
ax.set_yticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
ax.grid(which="minor", alpha=0.2)

painting = False
paint_value = None


def paint(event):
    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return

    x = int(round(event.xdata))
    y = int(round(event.ydata))

    if not (0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]):
        return

    grid[x, y] = paint_value

    img.set_data(grid.T)
    fig.canvas.draw_idle()


def on_press(event):
    global painting, paint_value

    if event.button == 1:
        paint_value = 99
    elif event.button == 3:
        paint_value = 0
    else:
        return

    painting = True
    paint(event)


def on_motion(event):
    if painting:
        paint(event)


def on_release(event):
    global painting
    painting = False


def on_key(event):
    global grid

    if event.key == "s":
        np.save(filename, grid)
        print(f"Saved changes to {filename}")
        print("Unique values:", np.unique(grid))

    elif event.key == "r":
        grid = original_grid.copy()
        img.set_data(grid.T)
        fig.canvas.draw_idle()
        print("Reset to original map")


fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("key_press_event", on_key)

plt.show()