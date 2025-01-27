"""
TODO:
AKtuell ist Spiegel falsch....gefühlt wäre es "richtig", wenn der mittlere Punkt einfach die Brennweite Koordinaten nach hinten gesetzt wird...
    - man sieht das akuell nicht so gut, weil der Spiegel noch nicht richtig ist: Aber wenn eins das manuell einstellt, und eins es dann leicht verstellt, damit es sich nicht überlappt: Verdoppelt das signal
- Die eine Linse ist noch zu klein? Wir haben jetzt einen inch, nicht 1/2 durchmesser. So halb gefixt, bin mir nicht sicher mit der Breite aktuell
- die Linse hinter dem Filter ist nicht richtig, sie braucht mehr Krümmung? Fokussiert aktuell nicht. Liegt wohl an der Rotation? Die Linsen sehen auch ein bisschen unterschiedlich aus...

- wegen der Linsen, weil das gerade so sehr auf plan konvexe angelegt ist: Es muss doch sehr einfach sein, dass für Linsen mit gegebener Brennweite und Länge zu berechnen?
- noch mal alles kontrollieren I guess, ob das stimmt...für später, könnte im Studium noch nützlich sein I guess, vielelicht
"""

import json
import math


class Point:
    def __init__(self, x, y, arc=False):
        self.x = x
        self.y = y
        self.arc = arc


def concave_mirror(p1, p2, focal_length):

    def midpoint(p1, p2):
        return Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)

    def distance(p1, p2):
        return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

    def radius_of_curvature(f):
        return 2 * f

    mid_point = midpoint(p1, p2)
    d = distance(p1, p2)
    R = radius_of_curvature(focal_length)
    h = math.sqrt(R**2 - (d / 2) ** 2)

    p3 = Point(mid_point.x - h, mid_point.y)

    return {
        "type": "ArcMirror",
        "p1": {"x": p1.x, "y": p1.y},
        "p2": {"x": p2.x, "y": p2.y},
        "p3": {"x": p3.x, "y": p3.y},
    }


def rotate_points(points, angle):
    angle_rad = math.radians(angle)
    rotated_points = []

    for p in points:
        x_rot = p.x * math.cos(angle_rad) - p.y * math.sin(angle_rad)
        y_rot = p.x * math.sin(angle_rad) + p.y * math.cos(angle_rad)
        rotated_points.append(Point(x_rot, y_rot, p.arc))

    return rotated_points


def points_lens(width, height, radius, cylin=0, centering=True):
    # based on the drawings provided by Thorlabs for Mounted Plano-Convex Round Cylindrical Lenses
    p1 = Point(0, -height / 2)
    p5 = Point(0, height / 2)
    p3 = Point(width, 0)
    # g_x(a) = cos(a) = x/r ; g_y(a) = sin(a) = y/r
    # y = r * sin(a) => a = arcsin(y/r)
    a = math.asin(p1.y / radius)  #     a = math.degrees(math.asin(p1.y / radius))
    # x = r * cos(a)
    p2 = Point(radius * math.cos(a), p1.y)
    # offset:
    # g_y(a) = sin(a) = y/r ; g_x(a) = cos(a) = x/r
    # x = r * cos(a) => a = arccos(x/r)
    a = math.acos((p1.x - (p3.x - radius)) / radius)
    x = radius * math.cos(a)
    p2.x -= x

    p4 = Point(p2.x, p5.y)

    # no cylindric lense
    if cylin == 0:
        p3 = Point(width, 0, True)
        p6 = Point(p1.x - (p3.x - p2.x), 0, True)
    # plano on right side
    elif cylin == 1:
        p3 = Point(p2.x, 0, False)
        p6 = Point(p1.x - (p3.x - p2.x), 0, True)
    # plano on left side
    elif cylin == 2:
        p3 = Point(width, 0, True)
        p6 = Point(0, 0, False)
    else:
        raise RuntimeError("cylin parameter is not set correctly")

    if centering:
        amount = (p2.x - p1.x) / 2
        p1.x -= amount
        p2.x -= amount
        p3.x -= amount
        p4.x -= amount
        p5.x -= amount
        p6.x -= amount
    return (p1, p2, p3, p4, p5, p6)


def place_point_ource(xy, scale=1, orign=(0, 0)):

    or_x, or_y = orign
    x, y = xy

    return {
        "type": "PointSource",
        "x": or_x + scale * x,
        "y": or_y + scale * y,
        "brightness": 0.10,
    }


def place_text(xy, label, scale=1, orign=(0, 0)):

    or_x, or_y = orign
    x, y = xy
    return {
        "type": "TextLabel",
        "x": or_x + scale * x,
        "y": or_y + scale * y,
        "text": label,
        "fontSize": 5,
        "fontStyle": "Italic",
        "alignment": "center",
    }


def place_beam_splitter(
    center, length, trans_ration=0.3, angle=45, scale=1, orign=(0, 0)
):

    or_x, or_y = orign
    c_x, c_y = center
    p1 = Point(-length / 2, 0)
    p2 = Point(length / 2, 0)
    rotaed = rotate_points((p1, p2), angle)
    p1 = rotaed[0]
    p1.x += c_x
    p1.y += c_y
    p2 = rotaed[1]
    p2.x += c_x
    p2.y += c_y

    return {
        "type": "BeamSplitter",
        "p1": {"x": or_x + scale * p1.x, "y": or_y + scale * p1.y},
        "p2": {"x": or_x + scale * p2.x, "y": or_y + scale * p2.y},
        "transRatio": trans_ration,
    }


def place_lense(offset, points, scale=1, orign=(0, 0), defBy="DFfdBfd"):
    of_x, of_y = offset
    or_x, or_y = orign

    path = [
        {
            "x": or_x + of_x + scale * point.x,
            "y": or_y + of_y + scale * point.y,
            "arc": point.arc,
        }
        for point in points
    ]
    spherical_lens = {"type": "SphericalLens", "path": path, "defBy": defBy}

    return spherical_lens


# [print(point) for point in points_lens(6.7, 21.5, 25.8)]


lense_cylin = points_lens(6.7, 21.5, 25.8, cylin=2)
# Thorlabs: The focal length of each lens can be calculated using a simplified thick lens equation: f = R/(n-1).
# Here _n_ is the index of refraction and _R_ is the radius of curvature of the lens surface. For more information, please see the _Tutorial_ tab.
# lense_ortho = points_lens(3, 11.4, 50)  # f == R ? Shouldn't it be f = 2R?
lense_ortho = points_lens(3, 25, 50)  # f == R ? Shouldn't it be f = 2R?
lense_hori = rotate_points(lense_ortho, 90)


elements = []

# elements.append(place_point_ource((0, 50)))
elements.append(place_point_ource((0, 0)))
# elements.append(place_point_ource((0, -50)))
elements.append(place_text((0, 10), label="Messobjekt"))

elements.append(place_lense((50, 0), lense_cylin))
elements.append(place_text((50, 20), label="Zylinderlinse"))
elements.append(place_lense((58, 0), lense_ortho))
elements.append(place_text((58, -20), label="Konvexlinse"))
elements.append(place_lense((0, -50), lense_hori))
elements.append(place_text((0, -40), label="Konvexlinse"))
elements.append(place_beam_splitter((0, -25), 20))
elements.append(place_text((0, -10), label="Filter"))

# idk ob die Funktion richtig ist, glaube, aber noch nicht geprüft
elements.append(concave_mirror(Point(-25, -40), Point(-25, 40), 20))
elements.append(place_text((-40, 60), label="Spiegel"))

# width als 2.0 mm für konvexe? (scaling in PDF richtig gesetzt (250/500 Prozzent) und dann mit Lineal am Bildschirm...1 cm als 1 mm)
# keine Ahnung wie eins das sonst ausrechnen soll, wie es bei den plan/konvexe ist. Eins kriegt einfach keine Infos zur width. KEINE

# place(lense2)

prefix = {"version": 5, "objs": []}
prefix["objs"] = elements
suffix = {
    "width": 1500,
    "height": 800,
    "rayModeDensity": 5,
    "origin": {"x": 0, "y": 0},
    "scale": 3,
    # "lockObjs": "true",
}
combined = {**prefix, **suffix}

with open("spherical_lens.json", "w") as json_file:
    json.dump(combined, json_file, indent=4)


with open("spherical_lens.json", "r") as file:
    import pyperclip

    pyperclip.copy(file.read())
