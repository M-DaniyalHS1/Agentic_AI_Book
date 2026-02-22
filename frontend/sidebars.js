// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    'intro',
    'what-is-physical-ai',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'doc',
        id: 'module-1/index',
      },
      items: [
        'module-1/ros2-architecture',
        'module-1/nodes-topics-services',
        'module-1/python-control',
        'module-1/urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      link: {
        type: 'doc',
        id: 'module-2/index',
      },
      items: [
        'module-2/gazebo-simulation',
        'module-2/unity-simulation',
        'module-2/physics-gravity',
        'module-2/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      link: {
        type: 'doc',
        id: 'module-3/index',
      },
      items: [
        'module-3/nvidia-isaac-sim',
        'module-3/synthetic-data',
        'module-3/isaac-ros-vslam',
        'module-3/nav2-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'doc',
        id: 'module-4/index',
      },
      items: [
        'module-4/speech-input-whisper',
        'module-4/llm-cognitive-planning',
        'module-4/natural-language-ros',
        'module-4/multimodal-perception',
      ],
    },
  ],
};

module.exports = sidebars;
