# This module defines the traffic signal phases for the robot

# Now hardcode default phase group on VPA map

# phase number is 2 digits, first digits say which intersection, second digit is phase index
# the main intersection is 1, so phases are 10, 11, 12, ...
# North : 2, South : 3, East : 4, West : 5

default_phase_group = {
    # Main Intersection, phase number 1x
    (300, 323): [12],
    (300, 319): [12],
    (300, 317): [12],
    (300, 310): [12],
    (300, 311): [12],
    (302, 315): [11], # right west to south
    (302, 319): [11], # straight west to west
    (303, 319): None, # reserved for future use
    (303, 317): [11], # straight west to east
    (304, 323): [14], # left west to north
    (305, 317): [13],
    (305, 319): [13],
    (305, 310): [13],
    (305, 311): [13],
    (305, 315): [13],
    (307, 323): [11], # right east to north
    (307, 310): [11], # straight east to east
    (308, 311): [11], # straight east to east
    (309, 315): [14],
    # West Intersection, phase number 2x
    (310, 325): [22],
    (310, 316): [23],
    (311, 316): [23],
    (312, 303): [22],
    (312, 304): [22],
    (312, 302): [22],
    (312, 316): [21,22], # straight north to south
    (313, 325): [21], # straight south to north
    (313, 302): [21],
    (313, 303): [21],
    (313, 304): [21],
    # South Intersection, phase number 3x
    (314, 313): [31],
    (314, 300): [31,32],
    (315, 313): [32,33,34],
    (315, 330): [33],
    (316, 330): [32,34],
    (316, 300): [34],
    # East intersection, phase number 4x
    (317, 333): [42],
    (318, 333): [41,43],
    (318, 309): [41],
    (318, 308): [41],
    (318, 307): [41],
    (319, 314): [41,42],
    (321, 314): [43],
    (321, 307): [43],
    (321, 308): [43],
    (321, 309): [43],
    # North intersection, phase number 5x
    (323, 321): [52],
    (323, 312): [53],
    (325, 321): [51,53],
    (325, 305): [51],
    (333, 312): [51],
    (333, 305): [52],

    # Special cases
    (330, 318): [0],
    (330, 331): [0], # special for exit the map
    (332, 318): [0], # special for entry the map
}

def get_phase_group_number(from_id, to_id):
    key = (from_id, to_id)
    if key in default_phase_group:
        return default_phase_group[key]
    else:
        return None