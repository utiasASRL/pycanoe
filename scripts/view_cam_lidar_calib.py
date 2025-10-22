import numpy as np

# import matplotlib.pyplot as plt
from pycanoe.data.sensors import Lidar, Camera
from pycanoe.utils.vis_utils import bilinear_interp
import open3d as o3d
import matplotlib.pyplot as plt

# path_lid = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-19-00/lidar/1755802821364780.bin"
# path_cam = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-19-00/cam_left/1755802821315769.png"

# path_lid = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-19-00/lidar/1755802844464735.bin"  # 44.464
# path_cam = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-19-00/cam_left/1755802844418450.png"  # 44.418
# path_cam_2 = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-19-00/cam_left/1755802844518383.png"  # 44.518

# path_lid = "/home/otter/otter_data_processed/canoe/extract_calib/canoe-2025-08-28-23-52/lidar/1756425170526283.bin"
# path_cam = "/home/otter/otter_data_processed/canoe/extract_calib/canoe-2025-08-28-23-52/cam_left/1756425170546748.png"

path_lid = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-11-31/lidar/1755775928359788.bin"  # 08.359
path_cam = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-11-31/cam_left/1755775928307903.png"  # 08.307


path_lid_save = "/home/otter/otter_data_processed/canoe/calib/lidar_overlay.png"
path_cam_save = "/home/otter/otter_data_processed/canoe/calib/cam_overlay.png"

T_sl_path = "/home/otter/otter_data_processed/calibration_info/T_sensor_lidar.txt"
# Updated uses the bumblebee intrinsics
T_cl_path = "/home/otter/otter_data_processed/calibration_info/T_cam_lidar-updated.txt"
P_c_path = "/home/otter/otter_data_processed/calibration_info/P_cam_left-updated.txt"


# ----- VIS FUNCTIONS ----- #
# region
def vis_points_on_image(
    img, uv, colors, im_show=True, color_bar=True, save=None, **kwargs
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
    if im_show:
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
    if color_bar:
        plt.colorbar(sc, ax=ax, fraction=0.03, pad=0)
    if save is not None:
        plt.savefig(save, bbox_inches="tight")
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

    # ----- LOAD TRANSFORMATIONS & DATA ----- #
    # region
    T_cam_lid = np.loadtxt(T_cl_path)
    T_sens_lid = np.loadtxt(T_sl_path)
    T_sens_lid[0:3, 3] *= 0.001  # m -> mm
    P_cam = np.loadtxt(P_c_path)

    # Data
    lidar = Lidar(path_lid)
    lidar.load_data()

    camera = Camera(path_cam)
    camera.load_data()
    # endregion

    # ----- TRANSFORM & FILTER DEPTH AND INTENSITY ----- #
    # region
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

    # 1) Lidar Points on Image w/ Colorized Depth
    vis_points_on_image(camera.img, uv, proj_colors, save=path_cam_save)
    # vis_points_on_image(camera.img, uv, uv_colors, im_show=False, color_bar=False, s=50)

    if False:
        lidar.visualize(
            fig_size=(15, 15), color_vec=uv_colors, azim_delta=0, elev_delta=0
        )

    # ----- OFFLOAD DATA ----- #
    lidar.unload_data()
    camera.unload_data()
