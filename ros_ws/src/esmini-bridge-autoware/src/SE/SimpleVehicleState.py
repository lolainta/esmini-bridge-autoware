import ctypes


class SESimpleVehicleState(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("h", ctypes.c_float),
        ("p", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("wheel_rotation", ctypes.c_float),
        ("wheel_angle", ctypes.c_float),
    ]
