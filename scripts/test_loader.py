import os.path as osp
import cv2
from tqdm import tqdm
from datetime import datetime, timezone
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from concurrent.futures import ProcessPoolExecutor

from pycanoe.data.sequence import Sequence
from pycanoe.data.sensors import Camera, Lidar, Radar, Sonar
from pycanoe.utils.utils import _get_closest_file_in_dir

matplotlib.use("Agg")

canoe_path = "/home/otter/otter_data_processed/canoe/"
seq_id = "canoe-2025-08-21-19-00"
seq_start = str(int(1755802837617698 + 120 * 1e6))
seq_end = str(int(1755802837617698 + 130 * 1e6))  # 30 seconds

# region: old args
cam_path = "/home/otter/otter_data_processed/canoe/canoe-2025-08-21-11-31/cam_left/1755780121273456.png"
# cam_path = "/home/otter/otter_data_processed/canoe/canoe-2025-08-28-14-13/cam_left/1756391243520667.png"
# cam_path = "/home/otter/otter_data_processed/canoe/canoe-2025-08-28-16-15/cam_left/1756398248869829.png"
# cam_path = "/home/otter/otter_data_processed/canoe/canoe-2025-08-20-18-07/cam_left/1755720150846574.png"

cam_show = False
lid_show = False
rad_show = False
son_show = True
# endregion


def write_cam_left_video(
    seq,
    out_path,
    img_shape=(2048, 1152),
    fps=100,  # cam 10hz, 100fps = 10x real time
    text_color=(0, 0, 0),
):

    cam_iter = seq.get_cam_left_iter()
    cam_total = len(seq.camleft_frames)

    writer = get_video_writer(out_path, img_shape, fps)

    font = cv2.FONT_HERSHEY_TRIPLEX

    for cam in tqdm(cam_iter, total=cam_total):

        # Load data
        utc_datetime = datetime.fromtimestamp(cam.timestamp, tz=timezone.utc)
        name = utc_datetime.strftime("%Y/%m/%d %I:%M:%S%p")

        # Plot
        img_bgr = cv2.cvtColor(cam.img, cv2.COLOR_RGB2BGR)
        cv2.putText(img_bgr, name, (10, 30), font, 1, text_color, 2, cv2.LINE_AA)

        writer.write(img_bgr)

    writer.release()


def get_video_writer(out_path, img_shape, fps):
    """
    Get video writer

    Args:
        out_path (str): path to save video
        img_shape (tuple): (width, height)
        fps (int): frames per second

    Returns:
        writer

    """
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, img_shape)

    return writer


def visualize_all_sensors_wrapper(frame_tuple):
    cam, rad, lid, son = frame_tuple
    return visualize_all_sensors(cam, rad, lid, son)


def visualize_all_sensors(cam, rad, lid, son, save=None):

    cam.load_data()
    rad.load_data()
    lid.load_data()
    son.load_data()

    utc_datetime = datetime.fromtimestamp(cam.timestamp, tz=timezone.utc)
    name = utc_datetime.strftime("%Y/%m/%d %I:%M:%S%p")

    fig, axs = plt.subplots(2, 2, figsize=(32, 23))

    # Steal content from each visualization
    ax_cam = cam.visualize(show=False)
    axs[0, 0].imshow(ax_cam.get_images()[0].get_array())
    axs[0, 0].axis("off")
    axs[0, 0].set_aspect("auto")
    plt.close(ax_cam.figure)

    ax_rad = rad.visualize(
        figsize=(11.52, 11.52),
        cart_resolution=rad.resolution * 4,
        cart_pixel_width=1152,
        show=False,
    )
    axs[0, 1].imshow(ax_rad.get_images()[0].get_array(), cmap="gray")
    axs[0, 1].axis("off")
    axs[0, 1].set_aspect("auto")
    plt.close(ax_rad.figure)

    ax_son = son.visualize(
        figsize=(20.48, 11.52),
        cart_pixel_height=1152,
        cart_resolution=son.resolution * 0.33,
        show=False,
    )
    axs[1, 0].imshow(ax_son.get_images()[0].get_array(), cmap="gray")
    axs[1, 0].axis("off")
    # axs[1,0].set_aspect('auto')
    plt.close(ax_son.figure)

    # Render lidar to image
    ax_lid = lid.visualize(
        figsize=(11.52, 11.52),
        # elev_delta=-120,
        # azim_delta=150,
        elev_delta=0,
        azim_delta=0,
        color="distance",
        show=False,
    )
    ax_lid.set_facecolor("black")
    ax_lid.figure.patch.set_facecolor("black")
    ax_lid.set_xlim([-100, 100])
    ax_lid.set_ylim([-100, 100])
    ax_lid.set_zlim([-3, 20])
    ax_lid.set_box_aspect((1, 1, 0.3))
    ax_lid.set_autoscale_on((False))
    ax_lid.view_init(elev=90, azim=161)
    #
    ax_lid.figure.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax_lid.figure.canvas.draw()
    lid_img = np.frombuffer(ax_lid.figure.canvas.tostring_rgb(), dtype=np.uint8)
    lid_img = lid_img.reshape(ax_lid.figure.canvas.get_width_height()[::-1] + (3,))
    axs[1, 1].imshow(lid_img)
    axs[1, 1].axis("off")
    # axs[1, 1].set_aspect("auto")
    plt.close(ax_lid.figure)

    fig.set_facecolor("black")
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0, hspace=0)

    fig.text(
        0.005,
        0.005,
        name,
        color="white",
        fontsize=48,
        verticalalignment="bottom",
        family="serif",
    )

    if save:
        plt.savefig(save)

    # Render to numpy array
    fig.canvas.draw()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    plt.close(fig)

    return img


def write_dashboard_video(
    seq,
    out_path,
    img_shape=(3200, 2300),
    fps=100,  # cam 10hz, 100fps = 10x real time
):

    writer = get_video_writer(out_path, img_shape, fps)
    total_frames = len(seq.camleft_frames)

    frame_tuples = list(
        zip(seq.camleft_frames, seq.radar_frames, seq.lidar_frames, seq.sonar_frames)
    )

    with ProcessPoolExecutor(max_workers=4) as executor:
        for img in tqdm(
            executor.map(visualize_all_sensors_wrapper, frame_tuples),
            total=total_frames,
        ):
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            writer.write(img_bgr)

    writer.release()


if __name__ == "__main__":

    seq = Sequence(canoe_path, [seq_id, seq_start, seq_end])
    seq.synchronize_frames(ref="cam_left")

    for i in range(2):
        cam = seq.camleft_frames[i]
        son = seq.sonar_frames[i]
        rad = seq.radar_frames[i]
        lid = seq.lidar_frames[i]
        visualize_all_sensors(cam, rad, lid, son, save=f"out-{i}.png")
    write_dashboard_video(seq, out_path="sens-5.mp4")
    # write_cam_left_video(seq, out_path="vid.mp4")

    """
    fig = plt.figure(figsize=figsize, dpi=dpi)
    ax = fig.add_subplot()
    ax.imshow(cam.img)
    ax.set_axis_off()
    ax.set_facecolor("black")
    return ax
    """

    if False:
        cam = Camera(cam_path)
        cam.load_data()
        if cam_show:
            cam.visualize(show=False, save="out.png")
        cam.unload_data()

        lid_folder = osp.join(cam.seq_root, "lidar")
        lid_file = _get_closest_file_in_dir(cam.timestamp, lid_folder)
        lid = Lidar(osp.join(lid_folder, lid_file))
        lid.load_data()
        if lid_show:
            lid.visualize(show=False, save="out.png")
        lid.unload_data()

        rad_folder = osp.join(cam.seq_root, "radar")
        rad_file = _get_closest_file_in_dir(cam.timestamp, rad_folder)
        rad = Radar(osp.join(rad_folder, rad_file))
        rad.load_data()
        if rad_show:
            rad.visualize(show=False, save="out.png")
        rad.unload_data()

        son_folder = osp.join(cam.seq_root, "sonar")
        son_file = _get_closest_file_in_dir(cam.timestamp, son_folder)
        son = Sonar(osp.join(son_folder, son_file))
        son.load_data()
        if son_show:
            son.visualize(show=False, save="out.png")
        son.unload_data()

    pass
