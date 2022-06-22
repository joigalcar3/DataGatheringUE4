import numpy as np
from math import *

# Computation of the acceleration due to the propeller failure
# Computation of the new mass
# Computation of the new Cg location

# User input
total_propeller_mass = 0.005
hub_mass = 0.001
blade_mass = (total_propeller_mass-hub_mass)/4

# Centrifugal force: F=mw^2r