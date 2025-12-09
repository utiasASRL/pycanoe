import cv2
from tqdm import tqdm
from datetime import datetime, timezone, timedelta
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from concurrent.futures import ProcessPoolExecutor

from pycanoe.data.sequence import Sequence

matplotlib.use("Agg")

UTC_TO_EDT_OFFSET = -5

canoe_path = "/home/otter/otter_data_processed/canoe/"
seq_id = "canoe-2025-08-21-19-00"
seq_start = str(int(1755802837617698 + 120 * 1e6))
seq_end = str(int(1755802837617698 + 130 * 1e6))  # 30 seconds


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

    utc_datetime = datetime.fromtimestamp(
        cam.timestamp, tz=timezone(timedelta(hours=UTC_TO_EDT_OFFSET))
    )
    name = utc_datetime.strftime("%Y/%m/%d %I:%M:%S%p")

    h1 = 1152
    h2 = 1152
    w1 = 2048
    w2 = 1152
    tot_img = np.zeros((h1 + h2, w1 + w2, 3), dtype=np.uint8)

    cam_img = cam.img  # (1152,2048,3)
    tot_img[0:h1, 0:w1, :] = cam_img

    rad_img = dash_radar(  # (1152,1152)
        rad,
        cart_resolution=rad.resolution * 4,
        cart_pixel_width=1152,
    )
    rad_img_rgb = (np.tile(rad_img[:, :, np.newaxis], 3) * 255).astype(np.uint8)
    tot_img[0:h1, w1 : w1 + w2, :] = rad_img_rgb

    son_img = dash_sonar(  # (1024,2048)
        son,
        cart_pixel_height=2048 // 2,
        cart_resolution=son.resolution * 0.38,
    )
    son_height = son_img.shape[0]
    son_img_rgb = (np.tile(son_img[:, :, np.newaxis], 3) * 255).astype(np.uint8)
    tot_img[h1 : h1 + son_height, 0:w1, :] = son_img_rgb

    lid_img = dash_lidar(lid, figsize=(11.52, 11.52), color="distance")  # (1152,152,3)
    tot_img[h2:, w1:, :] = lid_img

    img_bgr = cv2.cvtColor(tot_img, cv2.COLOR_RGB2BGR)

    cv2.putText(
        img_bgr,
        name,
        (20, h1 + h2 - 20),
        fontFace=cv2.FONT_HERSHEY_TRIPLEX,
        color=(255, 255, 255),
        fontScale=2.5,
        thickness=2,
    )

    if save:
        cv2.imwrite(save, img_bgr)

    return tot_img


def write_dashboard_video(
    seq,
    out_path,
    img_shape=(3200, 2304),
    fps=100,  # cam 10hz, 100fps = 10x real time
):
    seq.synchronize_frames(ref="cam_left")

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


def dash_lidar(
    lidar,
    figsize,
    cmap="jet",
    color="intensity",
):
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(projection="3d")

    p = lidar.points

    if color == "x":
        c = p[:, 0]
    elif color == "y":
        c = p[:, 1]
    elif color == "z":
        c = p[:, 2]
    elif color == "intensity":
        c = p[:, 3]
    elif color == "time":
        c = p[:, 4]
    elif color == "distance":
        c = np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
    else:
        print("warning: color: {} is not valid. Defaulting to 'z'".format(color))
        c = p[:, 2]

    xs = p[:, 0]
    ys = p[:, 1]
    zs = p[:, 2]

    ax.scatter(
        xs=xs,
        ys=ys,
        zs=zs,
        s=0.1,
        c=c,
        cmap=cmap,
        depthshade=False,
    )

    ax.set_xlim([-100, 100])
    ax.set_ylim([-100, 100])
    ax.set_zlim([-3, 20])
    ax.set_box_aspect((1, 1, 0.3))
    ax.set_autoscale_on((False))
    ax.view_init(elev=90, azim=161)

    ax.set_facecolor("black")
    ax.patch.set_facecolor("black")
    ax.figure.patch.set_facecolor("black")
    ax.grid(False)
    ax.xaxis.pane.set_visible(False)
    ax.yaxis.pane.set_visible(False)
    ax.zaxis.pane.set_visible(False)

    ax.figure.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax.figure.canvas.draw()
    lid_img = np.frombuffer(ax.figure.canvas.tostring_rgb(), dtype=np.uint8)
    lid_img = lid_img.reshape(ax.figure.canvas.get_width_height()[::-1] + (3,))
    # plt.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0, hspace=0)

    plt.close(fig)

    return lid_img


def dash_radar(
    rad,
    cart_resolution=None,
    cart_pixel_width=640,
):
    if cart_resolution is None:
        cart_resolution = rad.resolution * 5

    cart = rad.polar_to_cart(
        cart_resolution=cart_resolution,
        cart_pixel_width=cart_pixel_width,
        in_place=False,
    )
    return cart


def dash_sonar(
    son,
    cart_resolution=None,
    cart_pixel_height=320,
    use_polar=False,
):
    if cart_resolution is None:
        cart_resolution = son.resolution * 1.2

    if use_polar:
        im = son.polar
    else:
        im = son.polar_to_cart(
            cart_resolution=cart_resolution,
            cart_pixel_height=cart_pixel_height,
            in_place=False,
        )

    return im


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

    pass
