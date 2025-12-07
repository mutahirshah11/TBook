# Research: Part 4 - NVIDIA Isaac Platform

**Feature**: `011-part4-nvidia-isaac` | **Date**: 2025-12-07

## Decisions

### 1. Isaac Sim Installation on Linux
- **Decision**: Target **Ubuntu 20.04 LTS** (verified 22.04.5 with kernel 6.8+ support) with **NVIDIA Driver 535.129.03+**.
- **Rationale**:
  - Isaac Sim 4.0.0 has strict driver requirements. The 535.x branch is the recommended "Production Branch" driver for stability with Omniverse.
  - Linux is the industry standard for RL and Sim-to-Real workflows.
- **Alternatives Considered**:
  - *Windows*: While supported, it complicates the installation of RL libraries (Isaac Lab) which are Linux-native/optimized.
  - *Driver 525*: Older, might miss performance fixes in Isaac Sim 4.0.

### 2. Isaac Lab Installation Method
- **Decision**: Use the **Isaac Sim Pip Package** method within a dedicated **conda/venv** environment (Python 3.10).
- **Rationale**:
  - This is the "Recommended" modern workflow for Isaac Lab 4.0+.
  - Decouples the learning environment from the massive binary Isaac Sim installation, allowing for cleaner dependency management.
  - Steps: `conda create env`, `pip install isaacsim`, `git clone IsaacLab`, `./isaaclab.sh --install`.
- **Alternatives Considered**:
  - *Binary Installation*: More complex path linking, harder to manage python dependencies.

### 3. Replicator API for Synthetic Data
- **Decision**: Use the **Python API** (`omni.replicator.core`) for reproducibility.
- **Key Workflow**:
  - Initialize `simulation_app`.
  - Assign semantics via `rep.get.semantics.add_semantics(prim, "label")`.
  - Create `render_product` (Camera + Resolution).
  - Attach `AnnotatorRegistry.get_annotator("rgb" / "semantic_segmentation")`.
  - Use `rep.orchestrator.step()` to capture frames.
- **Rationale**: Programmatic generation allows for "headless" batch processing, which is essential for generating large datasets (e.g., 10k+ images) without manual UI interaction.

### 4. Domain Randomization Configuration
- **Decision**: Use **Python `@configclass`** with `EventTermCfg`.
- **Rationale**:
  - Isaac Lab has moved away from Hydra for *environment* configuration.
  - Domain Randomization (DR) events (mass, friction, color) are now defined as typed Python objects inside the environment config class.
  - This provides IDE autocompletion and type safety, unlike untyped YAML files.
- **Structure**:
  ```python
  @configclass
  class MyEnvCfg(DirectRLEnvCfg):
      events: EventCfg = EventCfg() # Inner class defining DR terms
  ```

## Unknowns Resolved

- **Driver Version**: Confirmed 535.129.03+.
- **RL Install**: Confirmed `pip install isaacsim` is the new standard.
- **DR Config**: Confirmed Python `EventTermCfg` replaces pure Hydra YAML for env logic.
- **Replicator**: Confirmed workflow requires explicit semantic labeling before annotator attachment.
