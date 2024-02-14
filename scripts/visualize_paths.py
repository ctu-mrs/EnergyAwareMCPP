import numpy as np
import matplotlib.pyplot as plt
import sys
import yaml
import math
from matplotlib.colors import hsv_to_rgb

METERS_IN_DEGREE = 111319.5


def gps_coordinates_to_meters(point: (float, float), lat_lon_origin: (float, float)) -> (float, float):
    """
    Transform points in gps latitude, longitude into meters relatively to lat_lon_origin
    :param point: point to transform
    :param lat_lon_origin: relative point to gent the position from
    """
    # TODO: This function ignores Earth curvature. This has to be replaced with a better calculation
    meters_in_long_degree = math.cos((lat_lon_origin[0] / 180.0) * math.pi) * METERS_IN_DEGREE
    return point[1] * meters_in_long_degree - lat_lon_origin[1] * meters_in_long_degree, point[0] * METERS_IN_DEGREE - \
           lat_lon_origin[0] * METERS_IN_DEGREE


def read_csv(filename: str, lat_lon_origin: (float, float) or None) -> np.array:
    """
    Read csv file into np.array transforming latitude,longitude into meters if needed
    :param lat_lon_origin: (latitude, longitude) origin if points are in (latitude, longitude), None if points are already in meters
    """
    res = np.loadtxt(filename, delimiter=",", dtype=np.float64)
    if lat_lon_origin is not None:
        res = np.apply_along_axis(lambda p: gps_coordinates_to_meters(p, lat_lon_origin), 1, res)
    return res


def main():
    if len(sys.argv) < 2:
        print("Error: too few arguments.\n usage: $ visualize_paths.py <config_filename.yaml> [<path.csv>]...")
        exit(-1)

    configuration = {}
    with open(sys.argv[1], "r") as f:
        try:
            configuration = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print("Error while parsing configuration file")
            print(e)
            exit(-1)

    # Set the origin for transformation to lat_lon origin from config file if one exists
    if configuration["points_in_lat_lon"]:
        lat_lon_origin = configuration["latitude_origin"], configuration["longitude_origin"]
    else:
        lat_lon_origin = None

    # Read all the needed files
    fly_zone_points = read_csv(configuration["fly_zone_filename"], lat_lon_origin)
    no_fly_zones_points = []
    paths_points = []
    if "no_fly_zones_filenames" in configuration:
        for filename in configuration["no_fly_zones_filenames"]:
            no_fly_zones_points.append(read_csv(filename, lat_lon_origin))

    for path_filename in sys.argv[2:]:
        paths_points.append(read_csv(path_filename, lat_lon_origin))

    # Plot the AOI and paths
    plt.figure()

    fly_zone_points = np.vstack([fly_zone_points, fly_zone_points[0]])
    plt.plot(fly_zone_points[:, 0], fly_zone_points[:, 1], color='green')

    for no_fly_zone in no_fly_zones_points:
        no_fly_zone = np.vstack([no_fly_zone, no_fly_zone[0]])
        plt.plot(no_fly_zone[:, 0], no_fly_zone[:, 1], color='red')

    for i in range(len(paths_points)):
        # Plot each path using a distinct color
        plt.plot(paths_points[i][:, 0], paths_points[i][:, 1],
                 color=hsv_to_rgb([0.2 + 0.8 * (i / len(paths_points)), 0.8, 0.8]))

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Visualization of AOI and generated paths")
    plt.axis('equal')

    plt.show()


if __name__ == "__main__":
    main()
