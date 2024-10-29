from math import pi, copysign

def orientation_solver(ori1, ori2):
    if copysign(1, ori1) == copysign(1, ori2):
        return ori1, ori2
    else:
        return ori1 + (2*pi * (ori1 < 0)), ori2 + (2*pi * (ori2 < 0))
