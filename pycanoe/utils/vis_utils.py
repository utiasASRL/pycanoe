import numpy as np
import matplotlib.pyplot as plt


# TODO: implement
def vis_camera(self, **kwargs):
    return NotImplementedError


# TODO: change defaults
def vis_lidar(
    lidar,
    fig_size=(10, 10),
    cmap="winter",
    color="intensity",
    color_vec=None,
    vmin=None,
    vmax=None,
    azim_delta=-75,
    elev_delta=-5,
    show=True,
    save=None,
):
    p = lidar.points

    # Choose color
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
    if color_vec is not None:
        c = color_vec
    # Min & max colors (only used if color_vec NOT set)
    if vmin is None or vmax is None:
        vmin = np.min(c)
        vmax = np.max(c)

    # Plot
    fig = plt.figure(figsize=fig_size)
    ax = fig.add_subplot(projection="3d")
    ax.azim += azim_delta
    ax.elev += elev_delta
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

    # Show & Save
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
