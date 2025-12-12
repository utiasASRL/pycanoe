import cv2
from tqdm import tqdm
from datetime import datetime, timezone, timedelta
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Pool, cpu_count
from pycanoe.data.sequence import Sequence

try:
    import open3d as o3d
except ModuleNotFoundError as e:
    print("WARNING:", e)

matplotlib.use("Agg")

UTC_TO_LOCAL = -4

canoe_path = "/home/otter/otter_data_processed/canoe/"
seq_id = "canoe-2025-08-21-19-00"
seq_start = "0"
seq_end = "9" * 16
# seq_start = str(int(1755802837617698 + 0 * 1e6))
# seq_end = str(int(1755802837617698 + 130 * 1e6))  # 30 seconds


_ax_cache = None


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
    # Try faster encoding then move to mp4
    # for codec in ["avc1", "x264", "H264", "mp4v"]:
    for codec in ["mp4v"]:
        fourcc = cv2.VideoWriter_fourcc(*codec)
        writer = cv2.VideoWriter(out_path, fourcc, fps, img_shape)
        if writer.isOpened():
            return writer

    raise RuntimeError("Could not create video writer with any codec")


def visualize_all_sensors(cam, rad, lid, son, mot):

    cam.load_data()
    rad.load_data()
    lid.load_data()
    son.load_data()
    mot.load_data()

    cam_img = cam.img  # (1152,2048,3)

    h1 = cam_img.shape[0]
    w1 = cam_img.shape[1]
    h2 = h1
    w2 = h2

    rad_img = dash_radar(  # (1152,1152)
        rad,
        cart_resolution=rad.resolution * 4,
        cart_pixel_width=w2,
    )
    rad_img_rgb = (np.tile(rad_img[:, :, np.newaxis], 3) * 255).astype(np.uint8)

    son_img = dash_sonar(  # (1024,2048)
        son,
        cart_pixel_height=w1 // 2,  # Sonar: h=w/2=w1/2
        cart_resolution=son.resolution * 0.38,
    )
    son_height = son_img.shape[0]
    son_img_rgb = (np.tile(son_img[:, :, np.newaxis], 3) * 255).astype(np.uint8)

    default_dpi = plt.rcParams["figure.dpi"]
    lid_img = dash_lidar(
        lid, figsize=(w2 / default_dpi, w2 / default_dpi), color="distance"
    )  # (1152,152,3)

    tot_img = np.zeros((h1 + h2, w1 + w2, 3), dtype=np.uint8)
    tot_img[0:h1, 0:w1, :] = cam_img
    tot_img[0:h1, w1 : w1 + w2, :] = rad_img_rgb
    tot_img[h1 : h1 + son_height, 0:w1, :] = son_img_rgb
    tot_img[h2:, w1:, :] = lid_img

    h3 = 80
    tot_img = np.vstack((tot_img, np.zeros((h3, tot_img.shape[1], 3), dtype=np.uint8)))
    draw_motor_bars(
        tot_img,
        abs(mot.port),
        abs(mot.starboard),
        abs(mot.max),
        int(tot_img.shape[1] * 0.4),
        52,
        int(tot_img.shape[1] * 0.107),
        margin=20,
        opacity=0.5,
    )

    cam.unload_data()
    rad.unload_data()
    lid.unload_data()
    son.unload_data()
    mot.unload_data()

    return tot_img


def visualize_all_sensors_wrapper(frame_tuple):
    cam, rad, lid, son, mot = frame_tuple

    return visualize_all_sensors(cam, rad, lid, son, mot), cam.timestamp


def visualize_cam_wrapper(frame_tuple):
    (cam,) = frame_tuple

    cam.load_data()
    return cam.img, cam.timestamp


def write_sequence_video(
    seq, out_path, n_workers=None, fps=100, cam_only=False, utc_to_local=-4
):

    if cam_only:
        frame_tuples = list(zip(seq.camleft_frames))
        wrapper = visualize_cam_wrapper
    else:
        seq.synchronize_frames(ref="cam_left")
        frame_tuples = list(
            zip(
                seq.camleft_frames,
                seq.radar_frames,
                seq.lidar_frames,
                seq.sonar_frames,
                seq.motor_frames,
            )
        )
        wrapper = visualize_all_sensors_wrapper

    img_shape = wrapper(frame_tuples[0])[0].shape[:2][::-1]  # dumb way to get (w,h)
    total = len(frame_tuples)

    if cam_only:
        vid_shape = img_shape
    else:
        vid_shape = (img_shape[0] // 2, img_shape[1] // 2)
    writer = get_video_writer(out_path, vid_shape, fps)

    if n_workers is None:
        n_workers = max(1, cpu_count() - 1)

    with Pool(processes=n_workers) as executor:

        for img, ts in tqdm(
            executor.imap(wrapper, frame_tuples, chunksize=2),
            total=total,
        ):
            ts_string = ts_string_from_utc(ts, utc_to_local=utc_to_local)

            margin = 20
            fontscale = 2.5
            fontthick = 2
            cv2.putText(
                img,
                ts_string,
                org=(margin, img.shape[0] - margin),
                fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                color=(255, 255, 255),
                fontScale=fontscale,
                thickness=fontthick,
            )

            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            img_bgr = cv2.resize(img_bgr, vid_shape)

            writer.write(img_bgr)

    writer.release()


def draw_motor_bars(
    img,
    left_power,
    right_power,
    max_power,
    bar_width,
    bar_height,
    text_width,
    margin=20,
    opacity=0.75,
):
    """
    Draw horizontal left and right power bars (green low, red high) at bottom right of image.

    Args:
        img: Image to modify in palce
        left_power: Left motor power (0 to max_power)
        right_power: Right motor power (0 to max_power)
        max_power: Max power value
        bar_width: Width of bar area
        bar_height: Height of bar area
        margin: Margin from bottom-right corner
        opacity: Opacity of the bars (0.0 to 1.0)
    """
    h, w = img.shape[:2]

    int_margin = 10

    # Position at bottom right with margin
    bar_x = w - bar_width - margin
    bar_y = h - bar_height - margin
    center_x = bar_x + bar_width // 2

    # Available width for bars (excluding text on both sides)
    avail_width = bar_width - 2 * text_width
    max_bar_width = avail_width // 2

    # Normalize power values
    left_norm = np.clip(left_power / max_power, 0, 1)
    right_norm = np.clip(right_power / max_power, 0, 1)

    # Calculate bar widths
    left_bar_width = int(left_norm * max_bar_width)
    right_bar_width = int(right_norm * max_bar_width)

    # Get colors from colormap (green->yellow->red)
    cmap = plt.colormaps["RdYlGn_r"]
    left_color = tuple((np.array(cmap(left_norm)[:3]) * 255).astype(int).tolist())
    right_color = tuple((np.array(cmap(right_norm)[:3]) * 255).astype(int).tolist())

    # Text
    text_l = f"L:{left_power:>4.0f}W"
    text_r = f"R:{right_power:>4.0f}W"

    # Create overlay for opacity
    overlay = img.copy()

    # Draw left bar (grows leftward from center)
    cv2.rectangle(
        overlay,
        (center_x - left_bar_width, bar_y),
        (center_x, bar_y + bar_height),
        left_color,
        -1,
    )

    # Draw right bar (grows rightward from center)
    cv2.rectangle(
        overlay,
        (center_x, bar_y),
        (center_x + right_bar_width, bar_y + bar_height),
        right_color,
        -1,
    )

    # Draw center line (full opacity)
    cv2.line(
        overlay,
        (center_x, bar_y - int_margin // 2),
        (center_x, bar_y + bar_height + int_margin // 2),
        # (center_x, bar_y),
        # (center_x, bar_y + bar_height),
        (255, 255, 255),
        2,
    )

    cv2.putText(
        overlay,
        text_l,
        # (bar_x + int_margin, bar_y + bar_height // 2 + int_margin // 2),
        (bar_x + int_margin, bar_y + bar_height),
        # (bar_x, bar_y + bar_height),
        fontFace=cv2.FONT_HERSHEY_TRIPLEX,
        color=(255, 255, 255),
        fontScale=2.5,
        thickness=2,
    )

    cv2.putText(
        overlay,
        text_r,
        (
            # bar_x + bar_width - text_width + int_margin,
            # bar_y + bar_height // 2 + int_margin // 2,
            bar_x + bar_width - text_width + int_margin,
            bar_y + bar_height,
        ),
        cv2.FONT_HERSHEY_TRIPLEX,
        color=(255, 255, 255),
        fontScale=2.5,
        thickness=2,
    )

    # Blend overlay with original image
    cv2.addWeighted(overlay, opacity, img, 1 - opacity, 0, img)


def ts_string_from_utc(ts_sec, utc_to_local):
    ts = datetime.fromtimestamp(ts_sec, tz=timezone(timedelta(hours=utc_to_local)))
    return ts.strftime("%Y/%m/%d %I:%M:%S%p")


# can't get o3d to work
def dash_lidar_fast(lidar, img_shape=(800, 800), color="z"):
    p = lidar.points
    width, height = img_shape

    color_options = {"x": 0, "y": 1, "z": 2, "intensity": 3, "time": 4}

    if color == "distance":
        c = np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)
    else:
        if color not in color_options.keys():
            print("WARNING: color: {} is not valid. Defaulting to 'z'".format(color))
            color = "z"

        idx = color_options[color]
        c = p[:, idx]

    # Normalize
    c = (c - c.min()) / (c.max() - c.min() + 1e-8)
    colors = plt.get_cmap("jet")(c)[:, :3]

    # Create 03d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(p[:, :3])
    cloud.colors = o3d.utility.Vector3dVector(colors)

    # Renderer
    render = o3d.visualization.rendering.OffscreenRenderer(width, height)
    render.scene.set_background([0, 0, 0, 1])  # black
    render.scene.add_geometry("pc", cloud, o3d.visualization.rendering.MaterialRecord())

    # camera
    center = np.array([0, 0, 2])
    eye = np.array([0, -7, 12])
    up = np.array([0, 0, 1])
    render.scene.camera.look_at(center, eye, up)

    img = render.render_to_image()
    return np.asarray(img)


def dash_lidar_cached(
    lidar,
    figsize,
    cmap="jet",
    color="z",
):
    global _ax_cache

    if _ax_cache is None:
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(projection="3d")
        _ax_cache = ax

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
    else:
        ax = _ax_cache
        ax.clear()

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

    ax.scatter(
        xs=p[:, 0],
        ys=p[:, 1],
        zs=p[:, 2],
        s=0.1,
        c=c,
        cmap=cmap,
        depthshade=False,
    )

    ax.figure.canvas.draw()

    buf = ax.figure.canvas.buffer_rgba()
    lid_img = np.frombuffer(buf, dtype=np.uint8)
    lid_img = lid_img.reshape(ax.figure.canvas.get_width_height()[::-1] + (4,))
    lid_img = lid_img[:, :, :3]

    return lid_img


def dash_lidar(
    lidar,
    figsize,
    cmap="jet",
    color="z",
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

    write_sequence_video(seq, out_path="dashboard.mp4", utc_to_local=UTC_TO_LOCAL)
    write_sequence_video(
        seq, out_path="cam.mp4", cam_only=True, utc_to_local=UTC_TO_LOCAL
    )
