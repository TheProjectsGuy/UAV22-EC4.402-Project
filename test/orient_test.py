# %% Import everything
import sympy as sp
import numpy as np

# %%
# Roll
def rot_x(angle, degrees = False):
    """
    Generates a Rotation matrice when the rotation is about X axis
    A general rotation about X (Roll) is given by
                | 1    0    0 |
    RotX(T) =   | 0   cT  -sT |
                | 0   sT   cT |
    Where T is rotation angle in radians
    Parameters:
    - angle: Symbol or float
        The angle of rotation
    - degrees: bool     default: False
        If 'True', then angle parameter is assumed to be in degrees
        else it is by default assumed to be in radians
    
    Returns:
    - rot_mat: sp.Matrix        shape: (3, 3)
        The 3x3 rotation matrix for Roll by angle_rad
    """
    angle_rad = angle if not degrees else sp.rad(angle)
    rot_mat = sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(angle_rad), -sp.sin(angle_rad)],
        [0, sp.sin(angle_rad), sp.cos(angle_rad)],
    ])
    return rot_mat


# Pitch
def rot_y(angle, degrees = False):
    """
    Generates a Rotation matrice when the rotation is about Y axis
    A general rotation about Y (Pitch) is given by
                |  cT   0   sT |
    RotY(T) =   |   0   1    0 |
                | -sT   0   cT |
    Where T is rotation angle in radians
    Parameters:
    - angle: Symbol or float
        The angle of rotation
    - degrees: bool     default: False
        If 'True', then angle parameter is assumed to be in degrees
        else it is by default assumed to be in radians
    
    Returns:
    - rot_mat: sp.Matrix        shape: (3, 3)
        The 3x3 rotation matrix for Pitch by angle_rad
    """
    angle_rad = angle if not degrees else sp.rad(angle)
    rot_mat = sp.Matrix([
        [sp.cos(angle_rad), 0, sp.sin(angle_rad)],
        [0, 1, 0],
        [-sp.sin(angle_rad), 0, sp.cos(angle_rad)],
    ])
    return rot_mat


# Yaw
def rot_z(angle, degrees = False):
    """
    Generates a Rotation matrice when the rotation is about Z axis
    A general rotation about Z (Yaw) is given by
                | cT   -sT   0 |
    RotZ(T) =   | sT    cT   0 |
                |  0     0   1 |
    Where T is rotation angle in radians
    Parameters:
    - angle: Symbol or float
        The angle of rotation
    - degrees: bool     default: False
        If 'True', then angle parameter is assumed to be in degrees
        else it is by default assumed to be in radians
    
    Returns:
    - rot_mat: sp.Matrix        shape: (3, 3)
        The 3x3 rotation matrix for Yaw by angle_rad
    """
    angle_rad = angle if not degrees else sp.rad(angle)
    rot_mat = sp.Matrix([
        [sp.cos(angle_rad), -sp.sin(angle_rad), 0],
        [sp.sin(angle_rad), sp.cos(angle_rad), 0],
        [0, 0, 1],
    ])
    return rot_mat


# %%
# Fixed XYZ (RPY) rotation angles (or Euler ZYX)
r, p, y = sp.symbols(r"\phi, \theta, \psi", real=True)

# %%
ori_vect = np.array([1., 1., 2.])
ori_vect = ori_vect/(np.linalg.norm(ori_vect)+1e-8)

# %%
z_vect = np.array([0.0, 0.0, 1.0])

# %%
rot_tf = rot_x(r) @ rot_y(p)

# %%
z = sp.Matrix(z_vect.reshape(-1, 1))
tx, ty, tz = sp.symbols(r"t_x, t_y, t_z", real=True)
o = sp.Matrix([[tx], [ty], [tz]])

# %%
sp.simplify(rot_tf @ o)

# %%
