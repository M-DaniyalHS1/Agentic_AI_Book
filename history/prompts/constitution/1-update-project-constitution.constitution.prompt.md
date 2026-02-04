---
id: 1
title: Update project constitution
stage: constitution
date_iso: 2026-02-04
surface: agent
model: Qwen
feature: none
branch: main
user: Qwen User
command: /sp.constitution CONSTITUTION
labels:
  - constitution
  - documentation
  - governance
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - .specify/memory/constitution.md
tests_yaml: []
---

# Update project constitution

## User Input

```
/sp.constitution CONSTITUTION
AI-Native Digital Textbook on Physical AI & Humanoid Robotics
1. Preamble

We, the contributors and maintainers of this project, establish this Constitution to guide the design, development, governance, and evolution of an AI-native digital textbook titled:

"Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World."

This project is developed under the educational vision of Panaversity and aims to:

Democratize access to cutting-edge Physical AI education

Integrate AI-driven authorship, simulation, and robotics learning

Pioneer AI-native interactive textbooks powered by RAG agents

Prepare learners for the future of human-AI-robot collaboration

2. Mission Statement

To build a living, AI-native, open, and interactive textbook that teaches:

Embodied Intelligence

Humanoid Robotics

ROS 2-based robot control

Physics simulation & digital twins

NVIDIA Isaac perception & navigation

Vision-Language-Action robotic cognition

while enabling AI-assisted learning through an embedded RAG chatbot.

3. Core Principles
3.1 AI-Native First

All content, tooling, and workflows must:

Support AI-assisted creation

Enable machine-readable structure

Allow agentic querying and reasoning

3.2 Open Knowledge & Accessibility

The book must be:

Openly accessible via GitHub Pages

Structured for self-learning

Designed for global students

3.3 Embodied Intelligence Focus

All curriculum must connect:

Digital AI → Physical World → Human Interaction

No purely theoretical AI without robotic embodiment relevance.

3.4 Reproducibility & Simulation-First

Every concept must include:

Code examples

Simulation workflows

Real-world deployment pathways

3.5 Modular & Extensible Design

The textbook must:

Be divided into clear modules

Support future AI/robotics updates

Allow community contributions

4. Scope of the Project
4.1 Educational Deliverable

A Docusaurus-based AI-native textbook containing:

Structured lessons

Code labs

Simulations

Visual diagrams

Assessments

4.2 Technical Deliverable

An embedded RAG chatbot that:

Answers questions about the book

Uses OpenAI Agents / ChatKit SDK

Runs on FastAPI backend

Stores embeddings in Qdrant Cloud

Uses Neon Serverless Postgres

Supports selected-text Q&A

4.3 AI-Driven Authoring

The book must be created using:

Spec-Kit Plus

Claude Code / Qwen CLI

Structured spec-driven writing workflows

5. Curriculum Constitution
Module 1 — The Robotic Nervous System (ROS 2)

Students must learn:

ROS 2 architecture

Nodes, Topics, Services

rclpy Python control

URDF humanoid modeling

Outcome:
Control a simulated humanoid via ROS 2.

Module 2 — The Digital Twin (Gazebo & Unity)

Students must learn:

Physics simulation

Gravity, collision, dynamics

Sensor simulation (LiDAR, depth, IMU)

Human-robot interaction environments

Outcome:
Create a realistic simulated world for robots.

Module 3 — The AI-Robot Brain (NVIDIA Isaac)

Students must learn:

Isaac Sim photorealistic environments

Synthetic data generation

Isaac ROS VSLAM & navigation

Nav2 path planning for humanoids

Outcome:
Enable autonomous perception and navigation.

Module 4 — Vision-Language-Action (VLA)

Students must learn:

Voice commands via Whisper

LLM cognitive planning

Natural language → ROS action pipelines

Capstone Outcome:
An Autonomous Humanoid Robot that:

Receives a voice command

Plans tasks using an LLM

Navigates obstacles

Detects objects via vision

Manipulates the environment

6. Technical Architecture Constitution
6.1 Frontend

Docusaurus

Hosted on GitHub Pages

AI-searchable structure

6.2 Backend (RAG System)

FastAPI

OpenAI Agents SDK

Qdrant Vector DB

Neon Postgres

6.3 AI Interaction Rules

The chatbot must:

Answer strictly from book knowledge

Cite relevant sections

Support context-aware tutoring

Enable selected-text explanation

7. Governance & Contribution
7.1 Maintainers

Primary maintainers are aligned with:

Panaversity leadership

Approved core contributors

7.2 Contribution Rules

All contributions must:

Follow Spec-Kit Plus structure

Maintain educational clarity

Include working code/simulations

Pass AI + human review

7.3 Ethical AI Use

The project forbids:

Unsafe robotics guidance

Harmful AI deployment instructions

Misleading scientific content

8. Quality Standards
8.1 Pedagogical Quality

Each chapter must include:

Learning objectives

Concept explanation

Hands-on lab

Visual aids

Assessment

8.2 Technical Quality

All code must be:

Runnable

Documented

Version-controlled

Simulation-verified

9. Evolution of the Constitution

This Constitution may evolve when:

Robotics technology advances

AI learning paradigms shift

Panaversity expands curriculum

Changes require:

Maintainer approval

Version update

Public documentation

10. Declaration

This Constitution establishes the foundation for building:

The world's first AI-native interactive textbook for Physical AI & Humanoid Robotics.

It represents a commitment to:

Open education

Embodied intelligence

Human-AI-robot collaboration

Future-ready learners
```

## Response Summary

Updated the project constitution at `.specify/memory/constitution.md` with concrete values based on the provided principles for the "AI-Native Digital Textbook on Physical AI & Humanoid Robotics" project. The constitution now contains six core principles and sections on Technical Architecture, Curriculum Framework, and Governance. A Sync Impact Report was added, and the document was validated for proper formatting and consistency. The version is set to 1.0.0, with a note that the RATIFICATION_DATE needs to be updated when known.