import numpy as np
import os
import open3d as o3d
import matplotlib.pyplot as plt

from pycanoe.data.sensors import Lidar, Camera
from pycanoe.utils.utils import get_time_from_filename, get_closest_index
from pycanoe.utils.vis_utils import bilinear_interp


# ----- DATA PATHS ----- #
cam_paths = [
    "~/otter_data_processed/canoe/canoe-2025-08-21-19-00/cam_left/1755802821315769.png",  # Boat 1a
    "~/otter_data_processed/canoe/canoe-2025-08-21-19-00/cam_left/1755802821415810.png",  # Boat 1b
    "~/otter_data_processed/canoe/canoe-2025-08-21-19-00/cam_left/1755802821515924.png",  # Boat 1c
    "~/otter_data_processed/canoe/canoe-2025-08-21-19-00/cam_left/1755802844418450.png",  # Boat 2
    "~/otter_data_processed/canoe/extract_calib/canoe-2025-08-28-23-52/cam_left/1756425170546748.png",  # Blerim Calib
    "~/otter_data_processed/canoe/canoe-2025-08-21-11-31/cam_left/1755775928307903.png",  # Shore
]

# ----- SAVE PATHS ----- #
save_path = os.path.expanduser(
    "~/otter_data_processed/calibration_info/visualizations/"
)

# region# ----- TRANSFORM PATHS ----- #
T_sl_path = "/home/otter/otter_data_processed/calibration_info/T_sensor_lidar.txt"
# Updated uses the bumblebee intrinsics
T_cl_path = "/home/otter/otter_data_processed/calibration_info/T_cam_lidar-updated.txt"
P_c_path = "/home/otter/otter_data_processed/calibration_info/P_cam_left-updated.txt"
# endregion

# region # ----- VIS FUNCTIONS ----- #


def vis_points_on_image(
    img, uv, colors, color_bar=True, place_text=None, show=True, save=None, **kwargs
):
    # Params
    height, width = img.shape[0:2]

    default_params = {
        "marker": ".",
        "s": 10,
        "edgecolors": "none",
        "alpha": 0.7,
        "cmap": "jet",
    }
    params = {**default_params, **kwargs}

    # Create figure
    fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
    ax = fig.add_subplot()
    ax.imshow(img)
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    sc = ax.scatter(
        uv[:, 0],
        uv[:, 1],
        c=colors,
        **params,
    )
    ax.set_axis_off()
    ax.invert_yaxis()
    if place_text is not None:
        plt.text(
            0.01,
            0.99,
            place_text,
            color="#800020",
            fontsize=18,
            ha="left",
            va="top",
            transform=plt.gca().transAxes,
        )
    if color_bar:
        plt.colorbar(sc, ax=ax, fraction=0.03, pad=0)
    if save is not None:
        plt.savefig(save, bbox_inches="tight")
    if show:
        plt.show()


def vis_lidar_o3d(lidar, colors=None, coord_frame=False, **kwargs):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar.points[:, :3])

    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    if coord_frame:
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=5.0,
            origin=[0, 0, 0],
        )
        o3d.visualization.draw_geometries([pcd, mesh_frame], **kwargs)
    else:
        o3d.visualization.draw_geometries([pcd], **kwargs)


# endregion


if __name__ == "__main__":

    # ----- DEBUG VARS ----- #
    max_depth = 6.0
    intensity_thresh = 0

    # region# ----- LOAD TRANSFORMATIONS ----- #
    T_cam_lid = np.loadtxt(T_cl_path)
    T_sens_lid = np.loadtxt(T_sl_path)
    T_sens_lid[0:3, 3] *= 0.001  # m -> mm
    P_cam = np.loadtxt(P_c_path)
    # endregion

    for cam_idx, cam_path in enumerate(cam_paths):

        # region# ----- LOAD DATA ----- #
        cam_path = os.path.expanduser(cam_path)
        camera = Camera(cam_path)
        camera.load_data()

        # Load closest lidar
        lidar_root = os.path.join(camera.seq_root, "lidar")
        lidar_files = os.listdir(lidar_root)
        lidar_times = [get_time_from_filename(name) for name in lidar_files]
        lidar_times, lidar_files = zip(*sorted(zip(lidar_times, lidar_files)))
        closest = get_closest_index(camera.timestamp, lidar_times)
        path_lid = os.path.join(lidar_root, lidar_files[closest])

        lidar = Lidar(path_lid)
        lidar.load_data()

        # Print time diff
        time_diff = abs(camera.timestamp - lidar.timestamp)
        print(f"Time Diff Pair {cam_idx}: {time_diff}")
        # endregion

        # region# ----- TRANSFORM & FILTER DEPTH AND INTENSITY ----- #
        # Transform: sensor -> lidar.
        lidar.transform(np.linalg.inv(T_sens_lid))
        # Transform: lidar -> camera.
        lidar.transform(T_cam_lid)

        # Filter for region of interest [xmin, xmax, ymin, ymax, zmin, zmax]
        # TODO: make all bounds customizable
        # bounds = []
        bounds = [-75, 75, -20, 20, 0.5, max_depth]
        # bounds = [-75, 75, -20, 20, 2, 50]
        lidar.passthrough(bounds)

        # Mask w/ intensity
        if intensity_thresh:
            lidar.points = lidar.points[(lidar.points[:, 3] > intensity_thresh)]
        # endregion

        # ----- LIDAR-IMAGE CORRESPONDENCES ----- #
        # Project onto image
        uv, proj_colors, uv_mask = lidar.project_onto_image(P_cam)
        uv_colors = bilinear_interp(camera.img, uv[:, 0], uv[:, 1]) / 255.0

        # ----- VISUALIZE ----- #
        # 0) Transform: camera -> lidar
        lidar.transform(np.linalg.inv(T_cam_lid))

        info_string = f"Time Diff: {time_diff:.4f}s\nMax Depth: {max_depth:.1f}m"
        # 1) Lidar Points on Image w/ Colorized Depth
        vis_points_on_image(
            camera.img,
            uv,
            proj_colors,
            place_text=info_string,
            show=False,
            save=os.path.join(save_path, "overlay-" + str(cam_idx) + ".png"),
        )
        # vis_points_on_image(camera.img, uv, uv_colors, im_show=False, color_bar=False, s=50)

        # ----- OFFLOAD DATA ----- #
        lidar.unload_data()
        camera.unload_data()
