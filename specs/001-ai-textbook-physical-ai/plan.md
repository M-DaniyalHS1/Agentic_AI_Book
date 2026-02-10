# Implementation Plan: AI-Native Digital Textbook on Physical AI & Humanoid Robotics

**Branch**: `001-ai-textbook-physical-ai` | **Date**: 2026-02-10 | **Spec**: [link to spec.md]
**Input**: Feature specification from `/specs/[001-ai-textbook-physical-ai]/spec.md`

## Summary

Develop a comprehensive AI-native digital textbook for Physical AI & Humanoid Robotics with an integrated RAG-based chatbot tutor and simulation environments. The system will be built using Docusaurus for the frontend, Python with FastAPI for the backend, Qdrant for vector storage, and PostgreSQL for structured data. The curriculum covers ROS 2, Digital Twins, AI-Robot Brains, and Vision-Language-Action systems with hands-on labs.

## Technical Context

**Language/Version**: Python 3.11 (for backend/FastAPI), JavaScript/TypeScript (for frontend/Docusaurus), with ROS 2 (likely Python/C++)
**Primary Dependencies**: 
- Frontend: React with Docusaurus framework
- Backend: Python with FastAPI, OpenAI Agents SDK, ChatKit
- Vector Database: Qdrant
- Database: PostgreSQL (Neon Serverless)
- Simulation: ROS 2, Gazebo, NVIDIA Isaac Sim, Unity
**Storage**: PostgreSQL for structured data (sessions, metadata), Qdrant for vector embeddings, Git for content management (Markdown files)
**Testing**: PyTest for backend, Jest for frontend, with simulation testing for robotics code
**Target Platform**: Web-based (hosted on GitHub Pages/Vercel), with simulation environments for robotics (Linux-based recommended for ROS/Gazebo)
**Project Type**: Web application (with frontend and backend components)
**Performance Goals**: 99% uptime during peak academic hours, responses within 3 seconds, search functionality within 30 seconds
**Constraints**: 
- Must support mobile and low-bandwidth connections
- AI responses must be grounded in textbook content only
- Citations required for all AI tutor responses
- Progressive enhancement (basic content available offline, advanced features online)
**Scale/Scope**: Designed to scale dynamically based on demand, with consistent performance; initially targeting undergraduate/graduate students, self-learners, and educators globally

## Constitution Check

Based on the AI-Native Digital Textbook on Physical AI & Humanoid Robotics Constitution, I'll evaluate the project against the core principles:

1. **AI-Native First**: ✓ The project incorporates an AI tutor with RAG system that understands the book content, supports "explain selected text" functionality, and enables agentic querying and reasoning.

2. **Open Knowledge & Accessibility**: ✓ The book is planned to be openly accessible via GitHub Pages, structured for self-learning, and designed for global students with mobile and low-bandwidth support.

3. **Embodied Intelligence Focus**: ✓ The curriculum connects Digital AI → Physical World → Human Interaction with modules specifically covering ROS 2, Gazebo simulations, NVIDIA Isaac Sim, and Vision-Language-Action pipelines.

4. **Reproducibility & Simulation-First**: ✓ Every concept includes code examples, simulation workflows, and real-world deployment pathways with hands-on labs and simulation exercises.

5. **Modular & Extensible Design**: ✓ The textbook is divided into clear modules (ROS 2, Digital Twins, AI-Robot Brain, Vision-Language-Action) and designed to support future updates and community contributions.

6. **Quality Education Standards**: ✓ Each chapter includes learning objectives, concept explanation, hands-on lab, visual aids, and assessment with runnable, documented, version-controlled code.

7. **Technical Architecture Compliance**: ✓ The architecture matches the constitution requirements with Docusaurus frontend, FastAPI backend, Qdrant vector DB, and PostgreSQL for metadata, with strict grounding of AI responses in textbook content.

All constitution principles are satisfied by the proposed implementation plan.

## Project Structure

### Documentation (this feature)

```text
specs/[001-ai-textbook-physical-ai]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application (when "frontend" + "backend" + "simulation" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── content_service.py
│   │   └── simulation_service.py
│   ├── api/
│   │   ├── content_router.py
│   │   ├── tutor_router.py
│   │   ├── session_router.py
│   │   └── assessment_router.py
│   └── utils/
│       ├── embedding_utils.py
│       └── validation_utils.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── scripts/
│   └── populate_embeddings.py
├── requirements.txt
├── alembic/
└── main.py

frontend/
├── src/
│   ├── components/
│   │   ├── TextbookViewer.jsx
│   │   ├── AITutorWidget.jsx
│   │   ├── SimulationViewer.jsx
│   │   └── AssessmentComponent.jsx
│   ├── pages/
│   ├── services/
│   │   ├── apiService.js
│   │   └── contentService.js
│   └── hooks/
│       └── useSessionTracking.js
├── tests/
├── docusaurus.config.js
├── sidebars.js
└── package.json

ros_ws/  # ROS 2 workspace for simulation components
├── src/
│   ├── digital_twin_module/
│   ├── ai_robot_brain_module/
│   └── vision_language_action_module/
├── launch/
├── config/
└── worlds/  # Gazebo simulation environments

content/
├── module_1_ros_nervous_system/
│   ├── chapter_1_introduction_to_ros.md
│   ├── chapter_2_nodes_topics_services.md
│   └── chapter_3_urdf_modeling.md
├── module_2_digital_twin/
│   ├── chapter_1_gazebo_simulation.md
│   └── chapter_2_sensor_simulation.md
├── module_3_ai_robot_brain/
│   ├── chapter_1_isaac_sim.md
│   └── chapter_2_navigation_planning.md
└── module_4_vision_language_action/
    ├── chapter_1_speech_recognition.md
    └── chapter_2_multimodal_control.md
```

**Structure Decision**: Selected the web application structure with separate backend and frontend components, plus a dedicated ROS workspace for simulation components. This structure allows for clear separation of concerns while maintaining the ability to integrate all components effectively. The content is stored separately in markdown files for easy management and version control.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple repositories | Need to separate concerns between web app and ROS simulation | Combining everything would create a monolithic, hard-to-maintain system | 
| Multiple databases | Different data types require different storage approaches | Single database would compromise performance for either transactional or vector data |