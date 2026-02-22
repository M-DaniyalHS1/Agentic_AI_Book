---
sidebar_position: 1
---

# Module 4: Vision-Language-Action

## Learning Objectives

By the end of this module, you will:
- Implement speech input using Whisper
- Use LLMs for cognitive planning
- Convert natural language to ROS actions
- Build multimodal perception-control loops

## The VLA Pipeline

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Speech    │ ──→ │    LLM      │ ──→ │   Action    │ ──→ │    ROS 2    │
│  (Whisper)  │     │  (Planning) │     │  Grounding  │     │  (Execution)│
└─────────────┘     └─────────────┘     └─────────────┘     └─────────────┘
       │                   │                   │                   │
       ↓                   ↓                   ↓                   ↓
  "Pick up the        "To pick up the      [reach(),          /arm/reach
   red ball"           ball, first         grasp(),           /gripper/close
                       reach, then         lift()]            /arm/lift
                       grasp"
```

## Module Structure

1. **Speech Input with Whisper** - Voice commands
2. **LLM Cognitive Planning** - Reasoning and decomposition
3. **Natural Language to ROS Actions** - Action grounding
4. **Multimodal Perception** - Vision + language integration

## Prerequisites

- Module 1-3 completed
- Python programming
- Basic understanding of neural networks
- ROS 2 proficiency
