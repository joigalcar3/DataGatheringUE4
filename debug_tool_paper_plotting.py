import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pickle
import os


mpl.use('TkAgg')
np.random.seed(1)

# Matplotlib settings
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42
mpl.rcParams['font.family'] = 'Arial'
mpl.rcParams['grid.alpha'] = 0.5
# mpl.use('Agg')
mpl.use('TkAgg')
font = {'size': 42,
        'family': "Arial"}
mpl.rc('font', **font)

folder = "F:\\Deleted items\\2020-2021\\Thesis project\\3_Execution_phase\\Simulator_images\\Debugging\\Positive_zero_correct_V_and_float_and_filter"
filename_1 = "(positions_y-positions_x-positions_z) vs (pos_ref_y-pos_ref_x-pos_ref_z).fig.pickle"
filename_2 = "front_left-front_right-back_right-back_left.fig.pickle"

location_save = "C:\\Users\\jialv\\OneDrive\\2020-2021\\Thesis project\\3_Execution_phase\\Thesis_report\\Images\\Corrected_blade_damage_plots"

directory_1 = os.path.join(folder, filename_1)
directory_2 = os.path.join(folder, filename_2)

with open(directory_1, "rb") as f:
    pickle.load(f)

with open(directory_2, "rb") as f:
    pickle.load(f)

labels = ["Actual flight", "Reference flight"]
plot_name = "debug_1.1"
fig = plt.figure(1)
ax = plt.gca()
plt.xlabel("y-coordinate [m]", fontsize=20, labelpad=20)
plt.ylabel("x-coordinate [m]", fontsize=20, labelpad=20)
ax.set_zlabel("z-coordinate [m]", fontsize=20, labelpad=25)
plt.title("")
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
ax.tick_params(labelsize=20)
ax.tick_params(axis="z", labelsize=20, pad=10)
leg = plt.legend(labels=labels, loc=1, fontsize=20)
for legobj in leg.legendHandles:
    legobj.set_linewidth(2.0)
fig.subplots_adjust(left=0.13, top=0.95, right=0.98, bottom=0.17)
fig.set_size_inches(19.24, 10.55)
fig.savefig(os.path.join(f"{location_save}", f"{plot_name}.pdf"), bbox_inches='tight')



labels = ["Front left", "Front right", "Back right", "Back left"]
plot_name = "debug_2"
fig = plt.figure(2)
plt.xlabel("Sample index [-]", fontsize=42)
plt.ylabel("$\\omega_{xc}$ [rad/s]", fontsize=42)
plt.title("")
plt.xticks(fontsize=42)
plt.yticks(fontsize=42)
leg = plt.legend(labels=labels, loc=1)
for legobj in leg.legendHandles:
    legobj.set_linewidth(4.0)
fig.subplots_adjust(left=0.13, top=0.95, right=0.98, bottom=0.17)
fig.set_size_inches(19.24, 10.55)
fig.savefig(os.path.join(f"{location_save}", f"{plot_name}.pdf"), bbox_inches='tight')

