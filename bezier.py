import math


class Vector2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def normalize(self):
        length = math.sqrt(self.x**2 + self.y**2)
        if length > 0:
            self.x /= length
            self.y /= length
        return self

    def __mul__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x * other.x, self.y * other.y)
        else:
            return Vector2(self.x * other, self.y * other)

    def __repr__(self):
        return f'({self.x}, {self.y})'

    def copy(self):
        return Vector2(self.x, self.y)


class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def rotate(self, angle, pivot):
        # Convert angle to radians
        angle_rad = math.radians(angle)

        # Translate point to origin
        translated_x = self.x - pivot.x
        translated_y = self.y - pivot.y

        # Rotate point
        rotated_x = translated_x * math.cos(angle_rad) - translated_y * math.sin(angle_rad)
        rotated_y = translated_x * math.sin(angle_rad) + translated_y * math.cos(angle_rad)

        # Translate point back
        self.x = rotated_x + pivot.x
        self.y = rotated_y + pivot.y

        return self

    def __add__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return Vector3(self.x + other, self.y + other, self.z + other)

    def __sub__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            return Vector3(self.x - other, self.y - other, self.z - other)

    def __mul__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        else:
            return Vector3(self.x * other, self.y * other, self.z * other)

    def __truediv__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x / other.x, self.y / other.y, self.z / other.z)
        else:
            return Vector3(self.x / other, self.y / other, self.z / other)

    def __repr__(self):
        return f'({self.x}, {self.y}, {self.z})'

    def distance_to(self, other):
        dx = other.x - self.x
        dy = other.y - self.y
        dz = other.z - self.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def copy(self):
        return Vector3(self.x, self.y, self.z)


def binomial_coefficient(n, k):
    result = 1

    # Calculate the binomial coefficient using the formula:
    # (n!) / (k! * (n - k)!)
    for i in range(1, k + 1):
        result *= n - (k - i)
        result //= i

    return result

on_get_point_on_bezier_curve: callable = None

def set_on_get_point_on_bezier_curve(callback: callable):
    global on_get_point_on_bezier_curve
    on_get_point_on_bezier_curve = callback


def get_point_on_bezier_curve(points, num_points, t, **kwargs):

    if isinstance(points[0], Vector2):
        pos = Vector2()

        for i in range(num_points):
            b = (
                binomial_coefficient(num_points - 1, i)
                * math.pow(1 - t, num_points - 1 - i)
                * math.pow(t, i)
            )
            pos.x += b * points[i].x
            pos.y += b * points[i].y

    elif isinstance(points[0], Vector3):
        pos = Vector3()

        for i in range(num_points):
            b = (
                binomial_coefficient(num_points - 1, i)
                * math.pow(1 - t, num_points - 1 - i)
                * math.pow(t, i)
            )
            pos.x += b * points[i].x
            pos.y += b * points[i].y
            pos.z += b * points[i].z
    else:
        raise TypeError('Points must be Vector2 or Vector3 objects')

    if on_get_point_on_bezier_curve:
        on_get_point_on_bezier_curve(points[0:num_points], t, result=pos, **kwargs)
    return pos

# Helper functions from Car_State.ino
def map_float(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def constrain(value, min_val, max_val):
    return max(min(value, max_val), min_val)


# Printing functions
def print_value(name, value, new_line=True):
    if isinstance(value, (Vector2, Vector3)):
        print(f'{name}: {str(value)}', end='\n' if new_line else '')
    else:
        print(f'{name}: {value}', end='\n' if new_line else '')
