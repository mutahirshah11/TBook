---
sidebar_label: 'Chapter 14: AI-powered perception and manipulation'
sidebar_position: 14
---

# Chapter 14: AI-Powered Perception and Manipulation

## Introduction

Traditional computer vision requires gathering thousands of real-world images and painstakingly annotating them by hand. This process is slow, expensive, and error-prone. In this chapter, we tackle the **Synthetic Data Gap** using Isaac Sim's **Replicator** engine.

We will learn to generate **Domain Randomized** datasets where lighting, texture, and object pose vary wildly. This prevents your AI model from "overfitting" to specific lighting conditions or background colors, making it robust when deployed in the real world.

## Setting up a Manipulation Scene

Before generating data, we need a 3D scene. We will set up a classic "Tabletop Manipulation" environment.

1.  **Launch Isaac Sim**: Use the `./isaac-sim.sh` script or the Launcher.
2.  **Add Robot**: Go to `Create > Isaac > Robots > Franka Panda`. This loads the 7-DOF arm with a gripper.
3.  **Add Physics**: Ensure the robot has an `ArticulationRoot` API applied. This tells the physics engine to treat the hierarchy of links and joints as a single constrained system.
4.  **Add Objects**: Create a simple table and a "target object" (e.g., a cube) for manipulation.
    - `Create > Shape > Cube`.
    - Scale it down to 0.05 (5cm).
    - **Crucial Step**: Add `RigidBody` and `Collision` APIs to the cube via the Property panel. Without these, the cube is just a "ghost" visual geometry that the robot will pass right through.

## Synthetic Data Generation with Replicator

**Replicator** is Isaac Sim's framework for programmatic data generation. It uses a "Graph" based approach: you define randomizers and triggers, and Replicator executes them efficiently at every frame.

### Code Example: `synthetic_data_gen.py`

This script generates RGB images and Semantic Segmentation masks for training a Neural Network.

```python
import omni.replicator.core as rep
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.world import World
import numpy as np

# 1. Initialize the Simulation World
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# 2. Define the Randomization Logic
def env_props(size=50):
    # Instantiate 'size' copies of the shapes
    instances = rep.randomizer.instantiate(
        rep.utils.get_usd_files("props/shapes"), 
        size=size, 
        mode='scene_instance'
    )
    
    # Apply Randomizers to these instances
    with instances:
        # Randomize Position within a bounding box
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, 0), (1, 1, 0)),
            rotation=rep.distribution.uniform((0,-180, 0), (0, 180, 0)),
        )
        # Randomize Color
        # 'diffuseColor' is the standard USD attribute for base color
        rep.modify.attribute("inputs:diffuseColor", rep.distribution.uniform((0,0,0), (1,1,1)))
        
        # Add Semantic Labels for Segmentation
        # This tags every pixel of these objects with the class 'shape'
        # The Annotator will read this tag to generate the mask.
        rep.modify.semantics([('class', 'shape')])
    return instances

# 3. Create the Sensor (Camera)
camera = rep.create.camera(position=(0, 0, 2), look_at=(0,0,0))
# A RenderProduct connects a camera to a resolution. 
# Multiple annotators can share the same RenderProduct for efficiency.
render_product = rep.create.render_product(camera, (1024, 1024))

# 4. Initialize Writers
# Writers handle saving the raw GPU data to disk formats (PNG, JSON, NumPy)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="_output_data",
    rgb=True,
    semantic_segmentation=True
)
writer.attach([render_product])

# 5. Trigger Generation
# Generate 10 frames of data. 
# Replicator will step the physics and randomizers automatically.
rep.orchestrator.run_until_frames(10)
```

## Annotators and Writers

Replicator separates data *capture* (Annotators) from data *storage* (Writers).

- **RGB Annotator**: Captures the standard lit color image.
- **Semantic Segmentation Annotator**: Renders a "False Color" image. It looks up the `semanticId` of the object at each pixel. For example, all pixels belonging to "Cube" might be rendered as solid red, while "Background" is black. This provides perfect, pixel-level labels for training segmentation networks (like Mask R-CNN).
- **BasicWriter**: Saves outputs to disk. For advanced users, you can write custom writers to convert data directly into **COCO** or **YOLO** formats, streamlining the pipeline to training tools.

### Deep Dive: The Semantic Schema
USD semantics are powerful. You can have multiple types of labels on the same object.
- `class`: General category (e.g., "car", "pedestrian").
- `color`: Specific attribute (e.g., "red", "blue").
Replicator allows you to filter which tag type the annotator should listen to. By default, it uses `class`.

## Visualizing Generated Data

After running the script, navigate to the `_output_data` folder. You will see pairs of images:
1.  `rgb_0001.png`: The photorealistic scene with random object positions.
2.  `semantic_segmentation_0001.png`: The segmentation mask.

**Pro Tip**: Use the `Colorize` setting in the writer or a python script to map the integer segmentation IDs to visible colors for easier debugging. By default, the IDs might be close to 0 (e.g., 1, 2, 3), making the image look pitch black to the naked eye unless normalized.

## Headless vs. UI Generation
While we ran this interactively, for large datasets (100k+ images), you should run Headless.
```bash
./isaac-sim.sh --no-window --exec synthetic_data_gen.py
```
This frees up GPU VRAM from rendering the UI, allowing you to spawn more assets or run parallel instances.
