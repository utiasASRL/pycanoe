import numpy as np


class PointCloud:
    """Class for working with lidar point clouds"""

    def __init__(self, points):
        self.points = points

    @staticmethod
    def get_aug(points):
        """Return new (N,4) array w/ added 1 column to each row vector."""
        return np.hstack((points[:, :3], np.ones((points.shape[0], 1))))

    def transform(self, T, in_place=True):
        """Transform points with transform T (x' = Tx).

        Args:
            T (np.ndarray): 4x4 transformation matrix
            in_place (bool): if True, self.points is updated

        Returns:
            points (np.ndarray): The transformed points
        """
        assert T.shape[0] == 4 and T.shape[1] == 4

        if in_place:
            points = self.points
        else:
            points = np.copy(self.points)

        # Transpose of x'=Tx equation (row vectors instead of column vectors)
        points[:, :3] = np.matmul(PointCloud.get_aug(points), T.transpose())[:, :3]
        return points

    def project_onto_image(
        self, P, width=2048, height=1152, color="depth", check_dims=True
    ):
        """Project 3D points onto a 2D image plane

        Args:
            P (np.ndarray): [fx 0 cx 0; 0 fy cy 0; 0 0 1 0; 0 0 0 1] cam projection
            width (int): width of image
            height (int): height of image
            color (str): 'depth' or 'intensity' to pick colors output
        Return:
            uv (np.ndarray): (N, 2) projected u-v pixel locations in an image
            colors (np.ndarray): (N,) a color value for each pixel location in uv.
            mask (np.ndarray): mask to select only points that project onto image.
        """
        x = PointCloud.get_aug(self.points)
        x /= x[:, 2:3]
        x[:, 3] = 1
        x = np.matmul(x, P.transpose())

        # Mask: only in-front of camera
        mask = self.points[:, 2] > 0
        # Mask: image dims
        if check_dims:
            mask *= (
                (x[:, 0] >= 0)
                & (x[:, 0] <= width - 1)
                & (x[:, 1] >= 0)
                & (x[:, 1] <= height - 1)
            )
        else:
            mask *= np.ones(x.shape[0], dtype=bool)
        x = x[mask]

        if color == "depth":
            colors = self.points[mask][:, 2]
        elif color == "intensity":
            colors = self.points[mask][:, 3]
        else:
            print("Warning: {} is not a valid color. Using depth.".format(color))
            colors = self.points[mask][:, 2]
        return x[:, :2], colors, mask

    def passthrough(self, bounds=[], in_place=True):
        """Remove points outside the specified bounds.

        Args:
            bounds (list): [xmin, xmax, ymin, ymax, zmin, zmax]
            in_place (bool): if True, self.points is updated

        Returns:
            points (np.ndarray): the remaining points after the filter is applied
        """
        if len(bounds) < 6:
            print("Warning: len(bounds) = {} < 6 is incorrect!".format(len(bounds)))
            return self.points

        mask = np.where(
            (self.points[:, 0] >= bounds[0])
            & (self.points[:, 0] <= bounds[1])
            & (self.points[:, 1] >= bounds[2])
            & (self.points[:, 1] <= bounds[3])
            & (self.points[:, 2] >= bounds[4])
            & (self.points[:, 2] <= bounds[5])
        )
        points_mask = self.points[mask]

        if in_place:
            self.points = points_mask
        return points_mask

    # TODO: Implement
    def remove_motion(self, body_rate, tref=None, in_place=True, single=False):
        raise NotImplementedError
