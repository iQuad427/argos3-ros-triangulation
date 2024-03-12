import math
import matplotlib.pyplot as plt


def circle_intersection(x1, y1, r1, x2, y2, r2):
    """Compute intersection points of two circles in 2D"""
    # Calculate distance between the centers of the circles
    dx = x2 - x1
    dy = y2 - y1
    dist = math.sqrt(dx*dx + dy*dy)

    # Check if circles are coincident or one circle is inside the other
    if dist > r1 + r2 or dist < abs(r1 - r2):
        # No intersection
        return []

    # Calculate intersection points
    a = (r1*r1 - r2*r2 + dist*dist) / (2 * dist)
    h = math.sqrt(r1*r1 - a*a)
    x3 = x1 + a * (x2 - x1) / dist
    y3 = y1 + a * (y2 - y1) / dist
    x4 = x3 + h * (y2 - y1) / dist
    y4 = y3 - h * (x2 - x1) / dist

    # If circles are touching at one point
    if dist == r1 + r2 or dist == abs(r1 - r2):
        return [(x4, y4)]
    else:
        # Two intersection points
        x5 = x3 - h * (y2 - y1) / dist
        y5 = y3 + h * (x2 - x1) / dist
        return [(x4, y4), (x5, y5)]


def main():
    circles = [
        (0, 0, 3),
        (0, 0, 4),
        (0, 10, 10),
        (0, 10, 9),
        (7, 6, 5),
        (7, 6, 6)
    ]

    ref_x, ref_y, uncertainty = 3, 1, 2

    # Compute all possible intersection points
    intersection_points = []
    for i in range(len(circles)):
        for j in range(i+1, len(circles)):
            circle1 = circles[i]
            circle2 = circles[j]
            intersection_points.extend(circle_intersection(*circle1, *circle2))

    # Take only the points inside the uncertainty circle
    intersection_points = [point for point in intersection_points if math.sqrt((point[0] - ref_x)**2 + (point[1] - ref_y)**2) < uncertainty]

    # Output the four closest intersection points
    print("Four closest intersection points:")
    for point in intersection_points[:]:
        print(point[0], point[1])

    ax = plt.gca()
    ax.cla()  # clear things for fresh plot

    # Axis aspect ratio should be equal
    ax.set_aspect('equal', adjustable='box')

    for circle in circles:
        ax.add_patch(plt.Circle((circle[0], circle[1]), circle[2], color='r', fill=False))

    # Plot a circle around the reference point
    ax.add_patch(plt.Circle((ref_x, ref_y), uncertainty, color='g', fill=False))

    for point in intersection_points[:]:
        ax.scatter(point[0], point[1], color='b')

    # Plot the reference point
    ax.scatter(ref_x, ref_y, color='g')

    # Plot the centroid of those points
    centroid_x = sum([point[0] for point in intersection_points[:]]) / len(intersection_points)
    centroid_y = sum([point[1] for point in intersection_points[:]]) / len(intersection_points)
    ax.scatter(centroid_x, centroid_y, color='r')

    # Plot vector from reference point to centroid
    ax.arrow(ref_x, ref_y, centroid_x - ref_x, centroid_y - ref_y, fc='k', ec='k')

    plt.show()


if __name__ == "__main__":
    main()
