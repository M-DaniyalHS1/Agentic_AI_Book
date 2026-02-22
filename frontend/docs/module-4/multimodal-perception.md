---
sidebar_position: 4
---

# Multimodal Perception

## Overview

Multimodal perception combines multiple sensory inputs for robust understanding:

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│    Vision    │     │    Language  │     │     Touch    │
│  (Camera)    │     │   (Speech)   │     │  (Force)     │
└──────────────┘     └──────────────┘     └──────────────┘
         │                  │                  │
         └──────────────────┼──────────────────┘
                            │
                   ┌────────▼────────┐
                   │  Multimodal     │
                   │  Fusion Layer   │
                   └────────┬────────┘
                            │
                   ┌────────▼────────┐
                   │  Unified Scene  │
                   │  Representation │
                   └─────────────────┘
```

## Vision-Language Models

### CLIP for Object Detection

```python
import clip
import torch
from PIL import Image

# Load CLIP model
model, preprocess = clip.load("ViT-B/32")

# Define text prompts
prompts = clip.tokenize([
    "a red ball",
    "a blue cup",
    "a wooden chair",
    "a white table"
])

# Load image
image = preprocess(Image.open("scene.jpg")).unsqueeze(0)

# Compute similarities
with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(prompts)
    
    similarities = torch.cosine_similarity(image_features, text_features)
    best_match = prompts[similarities.argmax()]
    
print(f"Detected: {best_match}")
```

### RGB-D Fusion

```python
import numpy as np

class RGBDFusion:
    def __init__(self):
        self.depth_scale = 0.001  # mm to meters
    
    def fuse(self, rgb_image, depth_image, camera_intrinsics):
        """Fuse RGB and depth into 3D point cloud"""
        height, width = rgb_image.shape[:2]
        
        # Create meshgrid of pixel coordinates
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        # Unproject to 3D
        fx, fy, cx, cy = camera_intrinsics
        z = depth_image * self.depth_scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # Stack into point cloud
        points = np.stack([x, y, z], axis=-1)
        colors = rgb_image / 255.0
        
        return points, colors
    
    def segment_by_color(self, points, colors, target_color):
        """Segment point cloud by color"""
        # Convert to HSV
        hsv = cv2.cvtColor(colors, cv2.COLOR_RGB2HSV)
        
        # Define color range
        lower = np.array([0, 100, 100])
        upper = np.array([10, 255, 255])
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Extract points
        segmented_points = points[mask > 0]
        return segmented_points
```

## Force-Vision Integration

```python
class ForceVisionFusion:
    def __init__(self):
        self.vision_subscriber = None
        self.force_subscriber = None
        self.contact_threshold = 5.0  # Newtons
    
    def detect_contact(self, force_reading, visual_estimate):
        """Detect contact using force and vision"""
        # Check force threshold
        force_magnitude = np.linalg.norm(force_reading)
        
        if force_magnitude > self.contact_threshold:
            return True, "Contact detected via force"
        
        # Check visual depth change
        if visual_estimate["distance_change"] < 0.01:  # 1cm
            return True, "Contact detected via vision"
        
        return False, "No contact"
    
    def estimate_object_weight(self, lift_force, gravity=9.81):
        """Estimate object weight from force readings"""
        # F = mg, so m = F/g
        mass = lift_force / gravity
        return mass
```

## Lab Exercise 4.4

Build a multimodal perception system:
1. Set up RGB-D camera
2. Implement CLIP-based detection
3. Fuse vision with force sensing
4. Test object manipulation tasks

## Course Complete! 🎉

You've completed all four modules of the Physical AI textbook. You now have the knowledge to:
- Build and control robots with ROS 2
- Simulate robots in digital twins
- Implement AI-powered navigation
- Create vision-language-action systems

Continue practicing with the hands-on labs and contribute to the open-source community!
