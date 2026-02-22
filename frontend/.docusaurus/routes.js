import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/',
    component: ComponentCreator('/', 'ff9'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '97d'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', '2b9'),
            routes: [
              {
                path: '/intro',
                component: ComponentCreator('/intro', '9fa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/',
                component: ComponentCreator('/module-1/', '61a'),
                exact: true
              },
              {
                path: '/module-1/nodes-topics-services',
                component: ComponentCreator('/module-1/nodes-topics-services', '263'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/python-control',
                component: ComponentCreator('/module-1/python-control', 'deb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/ros2-architecture',
                component: ComponentCreator('/module-1/ros2-architecture', '6b4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1/urdf-modeling',
                component: ComponentCreator('/module-1/urdf-modeling', '220'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/',
                component: ComponentCreator('/module-2/', 'c7a'),
                exact: true
              },
              {
                path: '/module-2/gazebo-simulation',
                component: ComponentCreator('/module-2/gazebo-simulation', '1eb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/physics-gravity',
                component: ComponentCreator('/module-2/physics-gravity', 'ad5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/sensor-simulation',
                component: ComponentCreator('/module-2/sensor-simulation', 'cf8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2/unity-simulation',
                component: ComponentCreator('/module-2/unity-simulation', '49e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/',
                component: ComponentCreator('/module-3/', '96a'),
                exact: true
              },
              {
                path: '/module-3/isaac-ros-vslam',
                component: ComponentCreator('/module-3/isaac-ros-vslam', '7bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/nav2-navigation',
                component: ComponentCreator('/module-3/nav2-navigation', '4b3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/nvidia-isaac-sim',
                component: ComponentCreator('/module-3/nvidia-isaac-sim', '071'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3/synthetic-data',
                component: ComponentCreator('/module-3/synthetic-data', 'f41'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/',
                component: ComponentCreator('/module-4/', 'dd2'),
                exact: true
              },
              {
                path: '/module-4/llm-cognitive-planning',
                component: ComponentCreator('/module-4/llm-cognitive-planning', 'e82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/multimodal-perception',
                component: ComponentCreator('/module-4/multimodal-perception', 'af2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/natural-language-ros',
                component: ComponentCreator('/module-4/natural-language-ros', 'd43'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4/speech-input-whisper',
                component: ComponentCreator('/module-4/speech-input-whisper', '5bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/what-is-physical-ai',
                component: ComponentCreator('/what-is-physical-ai', 'b05'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
