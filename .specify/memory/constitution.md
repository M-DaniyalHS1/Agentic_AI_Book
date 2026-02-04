<!--
Sync Impact Report:
Version change: N/A (initial version) → 1.0.0
Modified principles: None (new constitution)
Added sections: All sections are new to this initial version
Removed sections: None
Templates requiring updates: ✅ No updates needed / ⚠ pending - No templates required changes
Follow-up TODOs:
- RATIFICATION_DATE needs to be set to the original adoption date once known
-->
# AI-Native Digital Textbook on Physical AI & Humanoid Robotics Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### AI-Native First
<!-- Example: I. Library-First -->
All content, tooling, and workflows must support AI-assisted creation, enable machine-readable structure, and allow agentic querying and reasoning.
<!-- Example: Every feature starts as a standalone library; Libraries must be self-contained, independently testable, documented; Clear purpose required - no organizational-only libraries -->

### Open Knowledge & Accessibility
<!-- Example: II. CLI Interface -->
The book must be openly accessible via GitHub Pages, structured for self-learning, and designed for global students.
<!-- Example: Every library exposes functionality via CLI; Text in/out protocol: stdin/args → stdout, errors → stderr; Support JSON + human-readable formats -->

### Embodied Intelligence Focus
<!-- Example: III. Test-First (NON-NEGOTIABLE) -->
All curriculum must connect Digital AI → Physical World → Human Interaction. No purely theoretical AI without robotic embodiment relevance.
<!-- Example: TDD mandatory: Tests written → User approved → Tests fail → Then implement; Red-Green-Refactor cycle strictly enforced -->

### Reproducibility & Simulation-First
<!-- Example: IV. Integration Testing -->
Every concept must include code examples, simulation workflows, and real-world deployment pathways.
<!-- Example: Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas -->

### Modular & Extensible Design
<!-- Example: V. Observability, VI. Versioning & Breaking Changes, VII. Simplicity -->
The textbook must be divided into clear modules, support future AI/robotics updates, and allow community contributions.
<!-- Example: Text I/O ensures debuggability; Structured logging required; Or: MAJOR.MINOR.BUILD format; Or: Start simple, YAGNI principles -->

### Quality Education Standards
Each chapter must include learning objectives, concept explanation, hands-on lab, visual aids, and assessment. All code must be runnable, documented, version-controlled, and simulation-verified.

## Technical Architecture
<!-- Example: Additional Constraints, Security Requirements, Performance Standards, etc. -->

Frontend: Docusaurus hosted on GitHub Pages with AI-searchable structure. Backend (RAG System): FastAPI, OpenAI Agents SDK, Qdrant Vector DB, Neon Postgres. The chatbot must answer strictly from book knowledge, cite relevant sections, support context-aware tutoring, and enable selected-text explanation.
<!-- Example: Technology stack requirements, compliance standards, deployment policies, etc. -->

## Curriculum Framework
<!-- Example: Development Workflow, Review Process, Quality Gates, etc. -->

Module 1 — The Robotic Nervous System (ROS 2): Students learn ROS 2 architecture, Nodes/Topics/Services, rclpy Python control, URDF humanoid modeling. Module 2 — The Digital Twin (Gazebo & Unity): Physics simulation, gravity/collision/dynamics, sensor simulation, human-robot interaction environments. Module 3 — The AI-Robot Brain (NVIDIA Isaac): Isaac Sim environments, synthetic data generation, Isaac ROS VSLAM & navigation, Nav2 path planning. Module 4 — Vision-Language-Action (VLA): Voice commands via Whisper, LLM cognitive planning, natural language → ROS action pipelines.
<!-- Example: Code review requirements, testing gates, deployment approval process, etc. -->

## Governance
<!-- Example: Constitution supersedes all other practices; Amendments require documentation, approval, migration plan -->

Maintainers aligned with Panaversity leadership. All contributions must follow Spec-Kit Plus structure, maintain educational clarity, include working code/simulations, and pass AI + human review. The project forbids unsafe robotics guidance, harmful AI deployment instructions, and misleading scientific content. Constitution may evolve when robotics technology advances, AI learning paradigms shift, or Panaversity expands curriculum. Changes require maintainer approval, version update, and public documentation.
<!-- Example: All PRs/reviews must verify compliance; Complexity must be justified; Use .specify/memory/constitution.md for runtime development guidance -->

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown | **Last Amended**: 2026-02-04
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->
