# ROB6323 Go2 Project — Isaac Lab

This repository is the starter code for the NYU Reinforcement Learning and Optimal Control project in which students train a Unitree Go2 walking policy in Isaac Lab starting from a minimal baseline and improve it via reward shaping and robustness strategies. Please read this README fully before starting and follow the exact workflow and naming rules below to ensure your runs integrate correctly with the cluster scripts and grading pipeline.

## Repository policy

- Fork this repository and do not change the repository name in your fork.  
- Your fork must be named rob6323_go2_project so cluster scripts and paths work without modification.

### Prerequisites

- **GitHub Account:** You must have a GitHub account to fork this repository and manage your code. If you do not have one, [sign up here](https://github.com/join).

### Links
1.  **Project Webpage:** [https://machines-in-motion.github.io/RL_class_go2_project/](https://machines-in-motion.github.io/RL_class_go2_project/)
2.  **Project Tutorial:** [https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md](https://github.com/machines-in-motion/rob6323_go2_project/blob/master/tutorial/tutorial.md)

## Connect to Greene

- Connect to the NYU Greene HPC via SSH; if you are off-campus or not on NYU Wi‑Fi, you must connect through the NYU VPN before SSHing to Greene.  
- The official instructions include example SSH config snippets and commands for greene.hpc.nyu.edu and dtn.hpc.nyu.edu as well as VPN and gateway options: https://sites.google.com/nyu.edu/nyu-hpc/accessing-hpc?authuser=0#h.7t97br4zzvip.

## Clone in $HOME

After logging into Greene, `cd` into your home directory (`cd $HOME`). You must clone your fork into `$HOME` only (not scratch or archive). This ensures subsequent scripts and paths resolve correctly on the cluster. Since this is a private repository, you need to authenticate with GitHub. You have two options:

### Option A: Via VS Code (Recommended)
The easiest way to avoid managing keys manually is to configure **VS Code Remote SSH**. If set up correctly, VS Code forwards your local credentials to the cluster.
- Follow the [NYU HPC VS Code guide](https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code) to set up the connection.

> **Tip:** Once connected to Greene in VS Code, you can clone directly without using the terminal:
> 1. **Sign in to GitHub:** Click the "Accounts" icon (user profile picture) in the bottom-left sidebar. If you aren't signed in, click **"Sign in with GitHub"** and follow the browser prompts to authorize VS Code.
> 2. **Clone the Repo:** Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`), type **Git: Clone**, and select it.
> 3. **Select Destination:** When prompted, select your home directory (`/home/<netid>/`) as the clone location.
>
> For more details, see the [VS Code Version Control Documentation](https://code.visualstudio.com/docs/sourcecontrol/intro-to-git#_clone-a-repository-locally).

### Option B: Manual SSH Key Setup
If you prefer using a standard terminal, you must generate a unique SSH key on the Greene cluster and add it to your GitHub account:
1. **Generate a key:** Run the `ssh-keygen` command on Greene (follow the official [GitHub documentation on generating a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key)).
2. **Add the key to GitHub:** Copy the output of your public key (e.g., `cat ~/.ssh/id_ed25519.pub`) and add it to your account settings (follow the [GitHub documentation on adding a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)).

### Execute the Clone
Once authenticated, run the following commands. Replace `<your-git-ssh-url>` with the SSH URL of your fork (e.g., `git@github.com:YOUR_USERNAME/rob6323_go2_project.git`).
```
cd $HOME
git clone <your-git-ssh-url> rob6323_go2_project
```
*Note: You must ensure the target directory is named exactly `rob6323_go2_project`. This ensures subsequent scripts and paths resolve correctly on the cluster.*
## Install environment

- Enter the project directory and run the installer to set up required dependencies and cluster-side tooling.  
```
cd $HOME/rob6323_go2_project
./install.sh
```
Do not skip this step, as it configures the environment expected by the training and evaluation scripts. It will launch a job in burst to set up things and clone the IsaacLab repo inside your greene storage. You must wait until the job in burst is complete before launching your first training. To check the progress of the job, you can run `ssh burst "squeue -u $USER"`, and the job should disappear from there once it's completed. It takes around **30 minutes** to complete. 
You should see something similar to the screenshot below (captured from Greene):

![Example burst squeue output](docs/img/burst_squeue_example.png)

In this output, the **ST** (state) column indicates the job status:
- `PD` = pending in the queue (waiting for resources).
- `CF` = instance is being configured.
- `R`  = job is running.

On burst, it is common for an instance to fail to configure; in that case, the provided scripts automatically relaunch the job when this happens, so you usually only need to wait until the job finishes successfully and no longer appears in `squeue`.

## What to edit

- In this project you'll only have to modify the two files below, which define the Isaac Lab task and its configuration (including PPO hyperparameters).  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py  
  - source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env_cfg.py
PPO hyperparameters are defined in source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/agents/rsl_rl_ppo_cfg.py, but you shouldn't need to modify them.

## How to edit

- Option A (recommended): Use VS Code Remote SSH from your laptop to edit files on Greene; follow the NYU HPC VS Code guide and connect to a compute node as instructed (VPN required off‑campus) (https://sites.google.com/nyu.edu/nyu-hpc/training-support/general-hpc-topics/vs-code). If you set it correctly, it makes the login process easier, among other things, e.g., cloning a private repo.
- Option B: Edit directly on Greene using a terminal editor such as nano.  
```
nano source/rob6323_go2/rob6323_go2/tasks/direct/rob6323_go2/rob6323_go2_env.py
```
- Option C: Develop locally on your machine, push to your fork, then pull changes on Greene within your $HOME/rob6323_go2_project clone.

> **Tip:** Don't forget to regularly push your work to github

## Launch training

- From $HOME/rob6323_go2_project on Greene, submit a training job via the provided script.  
```
cd "$HOME/rob6323_go2_project"
./train.sh
```
- Check job status with SLURM using squeue on the burst head node as shown below.  
```
ssh burst "squeue -u $USER"
```
Be aware that jobs can be canceled and requeued by the scheduler or underlying provider policies when higher-priority work preempts your resources, which is normal behavior on shared clusters using preemptible partitions.

## Where to find results

- When a job completes, logs are written under logs in your project clone on Greene in the structure logs/[job_id]/rsl_rl/go2_flat_direct/[date_time]/.  
- Inside each run directory you will find a TensorBoard events file (events.out.tfevents...), neural network checkpoints (model_[epoch].pt), YAML files with the exact PPO and environment parameters, and a rollout video under videos/play/ that showcases the trained policy.  

## Download logs to your computer

Use `rsync` to copy results from the cluster to your local machine. It is faster and can resume interrupted transfers. Run this on your machine (NOT on Greene):

```
rsync -avzP -e 'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null' <netid>@dtn.hpc.nyu.edu:/home/<netid>/rob6323_go2_project/logs ./
```

*Explanation of flags:*
- `-a`: Archive mode (preserves permissions, times, and recursive).
- `-v`: Verbose output.
- `-z`: Compresses data during transfer (faster over network).
- `-P`: Shows progress bar and allows resuming partial transfers.

## Visualize with TensorBoard

You can inspect training metrics (reward curves, loss values, episode lengths) using TensorBoard. This requires installing it on your local machine.

1.  **Install TensorBoard:**
    On your local computer (do NOT run this on Greene), install the package:
    ```
    pip install tensorboard
    ```

2.  **Launch the Server:**
    Navigate to the folder where you downloaded your logs and start the server:
    ```
    # Assuming you are in the directory containing the 'logs' folder
    tensorboard --logdir ./logs
    ```

3.  **View Metrics:**
    Open your browser to the URL shown (usually `http://localhost:6006/`).

## Debugging on Burst

Burst storage is accessible only from a job running on burst, not from the burst login node. The provided scripts do not automatically synchronize error logs back to your home directory on Greene. However, you will need access to these logs to debug failed jobs. These error logs differ from the logs in the previous section.

The suggested way to inspect these logs is via the Open OnDemand web interface:

1.  Navigate to [https://ood-burst-001.hpc.nyu.edu](https://ood-burst-001.hpc.nyu.edu).
2.  Select **Files** > **Home Directory** from the top menu.
3.  You will see a list of files, including your `.err` log files.
4.  Click on any `.err` file to view its content directly in the browser.

> **Important:** Do not modify anything inside the `rob6323_go2_project` folder on burst storage. This directory is managed by the job scripts, and manual changes may cause synchronization issues or job failures.

## Project scope reminder

- The assignment expects you to go beyond velocity tracking by adding principled reward terms (posture stabilization, foot clearance, slip minimization, smooth actions, contact and collision penalties), robustness via domain randomization, and clear benchmarking metrics for evaluation as described in the course guidelines.  
- Keep your repository organized, document your changes in the README, and ensure your scripts are reproducible, as these factors are part of grading alongside policy quality and the short demo video deliverable.

## Resources

- [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/ecosystem.html) — Everything you need to know about IsaacLab, and more!
- [Isaac Lab ANYmal C environment](https://github.com/isaac-sim/IsaacLab/tree/main/source/isaaclab_tasks/isaaclab_tasks/direct/anymal_c) — This targets ANYmal C (not Unitree Go2), so use it as a reference and adapt robot config, assets, and reward to Go2.
- [DMO (IsaacGym) Go2 walking project page](https://machines-in-motion.github.io/DMO/) • [Go2 walking environment used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/tasks/go2_terrain.py) • [Config file used by the authors](https://github.com/Jogima-cyber/IsaacGymEnvs/blob/e351da69e05e0433e746cef0537b50924fd9fdbf/isaacgymenvs/cfg/task/Go2Terrain.yaml) — Look at the function `compute_reward_CaT` (beware that some reward terms have a weight of 0 and thus are deactivated, check weights in the config file); this implementation includes strong reward shaping, domain randomization, and training disturbances for robust sim‑to‑real, but it is written for legacy IsaacGym and the challenge is to re-implement it in Isaac Lab.
- **API References**:
    - [ArticulationData (`robot.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html#isaaclab.assets.ArticulationData) — Contains `root_pos_w`, `joint_pos`, `projected_gravity_b`, etc.
    - [ContactSensorData (`_contact_sensor.data`)](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html#isaaclab.sensors.ContactSensorData) — Contains `net_forces_w` (contact forces).

---
Students should only edit README.md below this line.


# ROB6323 Go2 Locomotion — Summary of Modifications

## Overview

This project extends the provided ROB6323 Go2 locomotion baseline environment. The original environment used joint position control and only rewarded linear and yaw velocity tracking. We introduced torque-level control, gait modeling, stability rewards, and advanced foot interaction modeling. Additional reward terms were added to further improve gait quality and realism.

---

## Major Changes and Rationale

### 1. Torque-Level PD Control *(Tutorial Part 1)*
**What changed**
- Replaced joint position targets with explicit PD torque control.
- Disabled Isaac Lab’s implicit actuator stiffness and damping.
- Added torque clipping and torque logging.

**Why**
- Torque control enables smoother and more realistic actuation, allows torque regularization, and aligns with modern quadruped locomotion pipelines.

**Where**
- `rob6323_go2_env.py`: `_pre_physics_step`, `_apply_action`
- `rob6323_go2_env_cfg.py`: actuator stiffness/damping set to zero, PD gains and torque limits defined

---

### 2. Action Rate Regularization *(Tutorial Part 1)*
**What changed**
- Added a rolling action history buffer (`last_actions`).
- Penalized both first-order (action rate) and second-order (action acceleration) differences.

**Why**
- Reduces jerky motions, improves smoothness, and stabilizes training.

**Where**
- `rob6323_go2_env.py`: `last_actions`, `rew_action_rate` in `_get_rewards`

---

### 3. Gait Phase Modeling and Clock Inputs *(Tutorial Part 4)*
**What changed**
- Implemented a periodic gait clock.
- Generated smooth stance/swing schedules for all four feet.
- Added gait phase signals (`clock_inputs`) to the observation space.

**Why**
- Encourages symmetric, periodic walking without hard-coding a fixed gait.

**Where**
- `rob6323_go2_env.py`: `_step_contact_targets`
- Observation vector expansion

---

### 4. Raibert Heuristic Foot Placement *(Tutorial Part 4)*
**What changed**
- Implemented Raibert-style foot placement error based on commanded velocity.
- Penalized deviation from desired stance locations.

**Why**
- Improves step placement and tracking of commanded motion.

**Where**
- `rob6323_go2_env.py`: `_reward_raibert_heuristic`

---

### 5. Posture and Motion Stabilization Rewards *(Tutorial Part 5)*
**What changed**
Added penalties for:
- Base tilt using projected gravity (XY components)
- Vertical base velocity (bouncing)
- Excessive joint velocities
- Roll and pitch angular velocities

**Why**
- Encourages upright posture, reduces oscillations, and improves overall stability.

**Where**
- `rob6323_go2_env.py`: `_get_rewards`
  - `orient`
  - `lin_vel_z`
  - `dof_vel`
  - `ang_vel_xy`

---

### 6. Foot Clearance During Swing *(Tutorial Part 6)*
**What changed**
- Penalized insufficient foot lift during swing phase.
- Used gait phase to gate the penalty to swing-only motion.

**Why**
- Prevents foot dragging and improves step quality.

**Where**
- `rob6323_go2_env.py`: `feet_clearance` reward

---

### 7. Contact Force Shaping During Swing *(Tutorial Part 6)*
**What changed**
- Used contact sensor force magnitudes.
- Penalized ground contact forces when a foot is expected to be in swing.
- Added lazy initialization of contact sensor body indices.

**Why**
- Encourages correct footfall timing and discourages spurious contacts.

**Where**
- `rob6323_go2_env.py`: `tracking_contacts_shaped_force`
- `rob6323_go2_env.py`: `_reset_idx` (sensor index initialization)

---

## Additional Reward Extensions (Beyond Tutorial)

These components were added beyond the official tutorial to further improve gait realism.

### 8. Torque Magnitude Penalty *(Extension)*

**What changed**
- Penalized the L2 norm of joint torques applied at each timestep.
- The penalty weight is defined in the environment configuration for easy tuning.

**Reward scale (defined in config)**
- `torque_reward_scale = -1e-4`

**Why**
- Encourages energy-efficient motion and avoids overly aggressive actuation.

**Where**
- `rob6323_go2_env_cfg.py`: `torque_reward_scale`
- `rob6323_go2_env.py`: `torque` reward applied in `_get_rewards`

---

### 9. Foot Slip (Sliding) Penalty *(Extension)*

**What changed**
- Penalized horizontal (XY) foot velocity during stance phases.
- Gated the penalty using the desired contact state from the gait planner.
- The penalty weight is defined in the environment configuration.

**Reward scale (defined in config)**
- `foot_slip_reward_scale = -0.005`

**Why**
- Reduces ground sliding, improves traction, and leads to cleaner foot contacts.

**Where**
- `rob6323_go2_env_cfg.py`: `foot_slip_reward_scale`
- `rob6323_go2_env.py`: `foot_slip` reward applied in `_get_rewards`

**Note**
This reward is **not explicitly required** by the tutorial and represents an additional extension built on the gait and contact modeling introduced there.

---

## TA-Recommended (in slack) Changes is Incorporated

### 10. Contact Sensor Registration (TA Fix)
**What changed**
- Explicitly registered the contact sensor with the scene.

**Why**
- Ensures contact force data is correctly updated and accessible for reward computation and termination checks. Without explicitly registering the sensor, contact-based rewards and terminations may use stale or missing data.

**Where**
- `rob6323_go2_env.py`: `_setup_scene`

---

### 11. Base Height Termination Threshold (TA Fix)
**What changed**
- Lowered the minimum base height threshold (`base_height_min`) from 0.20 m to 0.05 m.

**Why**
- The original 0.20m threshold caused premature episode termination during normal walking and crouching. Lowering the threshold allows natural base motion while still terminating collapsed or failed gaits.

**Where**
- `rob6323_go2_env_cfg.py`

---

## Termination Conditions
To improve sample efficiency and prevent training on unrecoverable states, episodes terminate if:
* **Base Collision:** The main body (chassis) makes contact with the ground.
* **Orientation:** The robot flips upside down.
* **Base Height:** The base height falls below 0.05 m.
* **Time Limit:** The maximum episode length is reached.

---

## Reward Summary

### Baseline Rewards (Starter Code)
* **track_lin_vel_xy_exp**: Exponential tracking of commanded linear velocity.
* **track_ang_vel_z_exp**: Exponential tracking of commanded yaw rate.

### Tutorial Rewards (Parts 1–6)
* **rew_action_rate**: Penalizes rapid and jerky action changes.
* **raibert_heuristic**: Encourages optimal foot placement based on current velocity.
* **orient**: Penalizes base roll and pitch to keep the robot level.
* **lin_vel_z**: Penalizes vertical "bouncing" motion.
* **dof_vel**: Penalizes excessive joint velocities.
* **ang_vel_xy**: Penalizes roll and pitch angular velocity.
* **feet_clearance**: Penalizes dragging feet during the swing phase.
* **tracking_contacts_shaped_force**: Penalizes ground contact forces when a foot should be in the air.

### Additional Rewards (Extensions)
* **torque**: Penalizes high torque magnitude for energy efficiency and motor safety.
* **foot_slip**: Penalizes horizontal foot velocity during stance to ensure firm traction and reduce sliding.

---

## Reproducibility

All reported experiments were run with a fixed random seed (42) to ensure reproducibility of environment initialization, command sampling, and reward computation.

### Training Command

Training was launched using the provided script with an explicit seed argument:

cd "$HOME/rob6323_go2_project"
./train.sh --seed 42

### Notes on Determinism
- The `--seed` flag initializes the random number generators used by Isaac Lab and the PPO training pipeline.
- All experiments in this repository use `seed = 42` unless otherwise noted.
- This ensures deterministic environment resets and command sampling across runs.
- Due to the stochastic nature of PPO (e.g., minibatch sampling and floating point nondeterminism), results are reproducible up to small numerical variation.