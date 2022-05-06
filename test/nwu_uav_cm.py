# The control matrix of a NWU UAV
"""
"""

# %% Import everything
import sympy as sp

# %%
kT, kt, L = sp.symbols(r"k_T, k_{\tau}, L", real=True)

# %%
M = sp.Matrix([
    [kT, kT, kT, kT],    # for T
    [0, L*kt, 0, -L*kt],    # for t_x
    [-L*kT, 0, L*kT, 0],    # for t_y
    [kt, -kt, kt, -kt]      # for t_z
])

# %%
