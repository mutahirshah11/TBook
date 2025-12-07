# Quickstart: Part 4 - NVIDIA Isaac Platform

**Prerequisites**:
- Linux System (Ubuntu 20.04/22.04)
- NVIDIA GPU (RTX 2070 or higher)
- NVIDIA Driver 535.129.03+ installed (`nvidia-smi` to verify)

## 1. Install Omniverse Launcher

1.  Download the **Omniverse Launcher** AppImage from the NVIDIA website.
2.  Make it executable: `chmod +x omniverse-launcher-linux.AppImage`
3.  Run it: `./omniverse-launcher-linux.AppImage`
4.  Log in with your NVIDIA Developer account.

## 2. Install Isaac Sim 4.0.0

1.  In the Launcher, go to the **Exchange** tab.
2.  Search for "Isaac Sim".
3.  Select Version **4.0.0**.
4.  Click **Install**. This download is large (~10GB+).

## 3. Setup Isaac Lab Environment

*We recommend using a dedicated python environment to keep dependencies clean.*

1.  Install Miniconda if you haven't already.
2.  Create the environment:
    ```bash
    conda create -n isaaclab python=3.10
    conda activate isaaclab
    ```
3.  Install Isaac Sim pip package:
    ```bash
    # Note: Isaac Sim must be installed via Launcher first
    # Source the Isaac Sim setup script to set environment variables
    source ~/.local/share/ov/pkg/isaac-sim-4.0.0/setup_python_env.sh
    ```
4.  Clone and Install Isaac Lab:
    ```bash
    git clone https://github.com/isaac-sim/IsaacLab.git
    cd IsaacLab
    ./isaaclab.sh --install
    ```

## 4. Verify Installation

Run the "Hello World" example to ensure the simulator launches and python bindings work.

```bash
# Inside IsaacLab directory
./isaaclab.sh -p source/stand alone/workflows/rsl_rl/train.py task=Isaac-Cartpole-v0 --headless
```

*Expected Output*: You should see logs indicating "PhysX initialized", "Loading Stage", and eventually a progress bar for training iterations.

## 5. Generate Synthetic Data (Replicator)

Run a simple script to verify Replicator.

```python
# save as test_rep.py
import omni.replicator.core as rep
# ... (Add basic camera setup code) ...
rep.orchestrator.step()
```

Execute with:
```bash
./isaaclab.sh -p test_rep.py
```
