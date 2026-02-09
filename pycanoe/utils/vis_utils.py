import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

try:
    import open3d as o3d  # noqa: F401
except ModuleNotFoundError as e:
    print("WARNING:", e)


def vis_camera(cam, figsize=(20.48, 11.52), dpi=100, show=True, save=None):
    fig = plt.figure(figsize=figsize, dpi=dpi)
    ax = fig.add_subplot()
    ax.imshow(cam.img)
    ax.set_axis_off()
    if show:
        plt.show()
    if save is not None:
        plt.savefig(save, bbox_inches="tight")
    return ax


def vis_lidar(
    lidar,
    figsize=(10, 10),
    cmap="jet",
    color="distance",
    color_vec=None,
    vmin=None,
    vmax=None,
    azim=160,
    elev=45,
    show=True,
    save=None,
):
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
        print("warning: color: {} is not valid. Defaulting to 'distance'".format(color))
        c = np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2)

    if color_vec is not None:
        c = color_vec
    if vmin is None or vmax is None:
        vmin = np.min(c)
        vmax = np.max(c)

    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(projection="3d")
    ax.azim = azim
    ax.elev = elev
    xs = p[:, 0]
    ys = p[:, 1]
    zs = p[:, 2]
    ax.set_box_aspect((np.ptp(xs), np.ptp(ys), np.ptp(zs)))
    ax.set_axis_off()

    if color_vec is None:
        ax.scatter(
            xs=xs,
            ys=ys,
            zs=zs,
            s=0.1,
            c=c,
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
            depthshade=False,
        )
    else:
        ax.scatter(xs=xs, ys=ys, zs=zs, s=0.1, c=c, depthshade=False)

    if show:
        plt.show()
    if save is not None:
        plt.savefig(save, bbox_inches="tight")

    return ax


def bilinear_interp(img, X, Y):
    """Retrieve interpolated pixel values from image at subpixel locations."""

    x = np.array(X).squeeze()
    y = np.array(Y).squeeze()

    x1 = np.floor(x).astype(np.int32)
    x2 = np.ceil(x).astype(np.int32)
    y1 = np.floor(y).astype(np.int32)
    y2 = np.ceil(y).astype(np.int32)

    mask = np.where(x1 == x2)

    q11 = img[y1, x1]  # N x 3
    q12 = img[y2, x1]
    q21 = img[y1, x2]
    q22 = img[y2, x2]

    EPS = 1e-14
    x_21 = x2 - x1 + EPS
    x_2 = ((x2 - x) / x_21).reshape(-1, 1)
    x_1 = ((x - x1) / x_21).reshape(-1, 1)

    f_y1 = q11 * x_2 + q21 * x_1
    f_y2 = q12 * x_2 + q22 * x_1

    f_y1[mask] = q11[mask]
    f_y2[mask] = q22[mask]

    mask = np.where(y1 == y2)

    y_21 = y2 - y1 + EPS
    y_2 = ((y2 - y) / y_21).reshape(-1, 1)
    y_1 = ((y - y1) / y_21).reshape(-1, 1)

    f = y_2 * f_y1 + y_1 * f_y2
    f[mask] = f_y1[mask]

    return f.squeeze()


def vis_radar(
    rad,
    figsize=(10, 10),
    dpi=100,
    cart_resolution=None,
    cart_pixel_width=640,
    cmap="gray",
    show=True,
    save=None,
):
    if cart_resolution is None:
        cart_resolution = rad.resolution * 5

    cart = rad.polar_to_cart(
        cart_resolution=cart_resolution,
        cart_pixel_width=cart_pixel_width,
        in_place=False,
    )
    fig = plt.figure(figsize=figsize, dpi=dpi)
    ax = fig.add_subplot()
    ax.imshow(cart, cmap=cmap)
    ax.set_axis_off()
    if show:
        plt.show()
    if save is not None:
        plt.savefig(save, bbox_inches="tight")
    return ax


def vis_sonar(
    son,
    figsize=(10, 5),
    dpi=100,
    cart_resolution=None,
    cart_pixel_height=320,
    cmap="gray",
    show=True,
    save=None,
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
    im = np.rot90(im, k=2)

    fig = plt.figure(figsize=figsize, dpi=dpi)
    ax = fig.add_subplot()
    ax.imshow(im, cmap=cmap)
    ax.set_axis_off()
    if show:
        plt.show()
    if save is not None:
        plt.savefig(save, bbox_inches="tight")
    return ax


def vis_lidar_o3d(lidar, colors=None, coord_frame=False, **kwargs):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar.points[:, :3])

    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    if coord_frame:
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=3.0,
            origin=[0, 0, 0],
        )
        o3d.visualization.draw_geometries([pcd, mesh_frame], **kwargs)
    else:
        o3d.visualization.draw_geometries([pcd], **kwargs)
