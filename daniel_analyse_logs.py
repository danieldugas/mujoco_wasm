import numpy as np
import matplotlib.pyplot as plt
import os

# load ~/mujoco_one_qlog.csv
with open(os.path.expanduser('~/mujoco_one_qlog.csv'), 'r') as f:
    lines = f.readlines()
    lines = [line.strip().split(',') for line in lines]
    lines = [[float(x) for x in line] for line in lines]
    mujoco_one = np.array(lines)

# load ~/wasm_one_qlog.csv
with open(os.path.expanduser('~/wasm_one_qlog.csv'), 'r') as f:
    lines = f.readlines()
    lines = [line.strip().split(',') for line in lines]
    lines = [[float(x) for x in line if x != ''] for line in lines]
    wasm_one = np.array(lines)

# top bottom subplots, with first 8 values
fig, axs = plt.subplots(2, 1, sharex=True)
fig.suptitle('Target obs')
for i in range(8):
    axs[0].plot(mujoco_one[:, i], label=f'mujoco_one_{i}')
    axs[1].plot(wasm_one[:, i], label=f'wasm_one_{i}')
axs[0].legend()
axs[1].legend()

joint_name_list = ["abdomen_z", "abdomen_y", "abdomen_x", "right_hip_x", "right_hip_z", "right_hip_y", "right_knee", "left_hip_x", "left_hip_z", "left_hip_y", "left_knee", "right_shoulder1", "right_shoulder2", "right_elbow", "left_shoulder1", "left_shoulder2", "left_elbow"]

# next 34 values (every other) (joints)
fig, axs = plt.subplots(2, 1, sharex=True)
fig.suptitle('Joint obs')
idx = 0
for i in range(8, 8+34, 2):
    axs[0].plot(mujoco_one[:, i], label=f'mujoco_one_{joint_name_list[idx]}')
    axs[1].plot(wasm_one[:, i], label=f'wasm_one_{joint_name_list[idx]}')
    idx += 1
axs[0].legend()
axs[1].legend()

# interstitial values (joint velocities)
fig, axs = plt.subplots(2, 1, sharex=True)
fig.suptitle('Joint velocities')
idx = 0
for i in range(9, 9+34, 2):
    axs[0].plot(mujoco_one[:, i], label=f'mujoco_one_{joint_name_list[idx]}')
    axs[1].plot(wasm_one[:, i], label=f'wasm_one_{joint_name_list[idx]}')
    idx += 1

# next 2 values (foot contacts)
fig, axs = plt.subplots(2, 1, sharex=True)
fig.suptitle('Foot contacts')
for i in range(42, 42+2):
    axs[0].plot(mujoco_one[:, i], label=f'mujoco_one_{i}')
    axs[1].plot(wasm_one[:, i], label=f'wasm_one_{i}')
axs[0].legend()
axs[1].legend()

# remaining values (actions)
fig, axs = plt.subplots(2, 1, sharex=True)
fig.suptitle('Actions')
for i in range(44, 44+8):
    axs[0].plot(mujoco_one[:, i], label=f'mujoco_one_{i}')
    axs[1].plot(wasm_one[:, i], label=f'wasm_one_{i}')
axs[0].legend()
axs[1].legend()


plt.show()

raise ValueError("STOP")