---
sidebar_position: 2
---

# Synthetic Data Generation

## Why Synthetic Data?

Training AI models requires large datasets. Synthetic data offers:
- Unlimited labeled data
- Perfect ground truth
- Edge case coverage
- No privacy concerns
- Cost-effective scaling

## Isaac Sim Replicator

```python
import omni.replicator.core as rep

# Register assets
rep.register_assets("/path/to/assets")

# Define randomization
with rep.new_layer():
    # Random camera positions
    camera = rep.create.camera()
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((0, 0, 0), (5, 5, 5)),
            look_at=rep.distribution.uniform((-2, -2, -2), (2, 2, 2))
        )
    
    # Random objects
    objects = rep.create.object(
        n=rep.distribution.uniform(1, 10)
    )
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((-5, 0, -5), (5, 2, 5))
        )
    
    # Random lighting
    lights = rep.create.light(
        light_type="sphere",
        intensity=rep.distribution.uniform(100, 1000)
    )
    with lights:
        rep.modify.pose(
            position=rep.distribution.uniform((-5, 5, -5), (5, 10, 5))
        )

# Configure writers
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/output/data",
    rgb=True,
    depth=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    bounding_box_2d_tight=True,
    bounding_box_3d=True
)

# Run generation
rep.run_sync(1000)  # Generate 1000 frames
```

## Data Formats

### RGB Images
- Standard color images
- Format: PNG or JPEG
- Resolution: Configurable

### Depth Maps
- Per-pixel distance
- Format: EXR or PNG (16-bit)
- Units: Meters

### Semantic Segmentation
- Per-pixel class labels
- Format: PNG with color palette
- Classes: Configurable

### Bounding Boxes
- 2D and 3D boxes
- Format: COCO JSON
- Includes: Class, position, size, rotation

## Lab Exercise 3.2

Generate a dataset for object detection:
1. Create 10 object types
2. Randomize positions and lighting
3. Generate 10,000 images
4. Export in COCO format

## Next Steps

Learn about Visual SLAM with Isaac ROS.
