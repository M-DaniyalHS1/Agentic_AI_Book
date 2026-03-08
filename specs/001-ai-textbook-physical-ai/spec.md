# Feature Specification: AI-Native Digital Textbook on Physical AI & Humanoid Robotics

**Feature Branch**: `001-ai-textbook-physical-ai`
**Created**: 2026-02-05
**Status**: Draft
**Input**: User description: "Product Specification AI-Native Digital Textbook on Physical AI & Humanoid Robotics Spec Version: 1.0.0 Status: Draft (Hackathon Submission Target) Applies To: Book Content, Platform, RAG System, Tooling, Governance Constitution: AI-Native Digital Textbook on Physical AI & Humanoid Robotics Constitution v1.0.0 1. Problem Statement Modern AI education treats software intelligence and physical embodiment as separate disciplines. Students learn LLMs without robots, or robotics without modern AI cognition. This gap prevents learners from understanding and building Physical AI systems where perception, reasoning, action, and human interaction are unified. Additionally: Traditional textbooks are static, non-interactive, and non-AI-native Robotics learning lacks reproducibility, simulation-first workflows, and agentic tutoring Learners cannot easily ask "why did this robot do that?" at the code + concept level 2. Product Vision Create the world's first AI-native digital textbook for Physical AI & Humanoid Robotics, where: 📘 The book is machine-readable 🤖 An AI tutor understands the book better than any human TA 🧠 Students learn Digital AI → Physical World → Human Interaction 🧪 Every concept is simulatable, runnable, and reproducible 🌍 Knowledge is open, global, and community-extensible This project serves as: A flagship Panaversity book A reference implementation for future AI-native textbooks A launchpad for startup-grade AI education products 3. Target Users Primary Undergraduate & graduate students (CS, AI, Robotics) Self-learners entering Physical AI / Robotics Panaversity, PIAIC, GIAIC cohorts Secondary Robotics engineers transitioning to AI AI engineers transitioning to robotics Educators building AI-native curricula 4. Core Deliverables 4.1 AI-Native Digital Textbook Built with Docusaurus Deployed on GitHub Pages Structured as machine-readable modules Includes: Learning objectives Concept explanations Diagrams & visuals Hands-on labs Simulation exercises Assessments 4.2 Integrated RAG Chatbot Embedded directly into the book UI Answers questions strictly from book content Can explain: Entire chapters Specific sections User-selected text Supports: Context-aware tutoring Step-by-step explanations Citation of exact book sections 4.3 Reproducible Robotics Codebase ROS 2 Gazebo / Unity NVIDIA Isaac Sim Vision-Language-Action pipelines Fully runnable and documented 5. Non-Goals (Explicit) ❌ No proprietary / closed content ❌ No unsafe real-world robotics instructions ❌ No purely theoretical AI without embodiment ❌ No hallucinating chatbot answers ❌ No black-box simulations without explanation 6. Functional Requirements 6.1 Book Platform RequirementDescription Static HostingGitHub Pages FrameworkDocusaurus NavigationModule → Chapter → Section SearchAI-searchable + keyword search VersioningBook versions tied to git tags AccessibilityMobile + low-bandwidth friendly 6.2 Curriculum Structure Each module MUST include: Learning Objectives Conceptual Explanation Visual Aids (diagrams, flows) Hands-On Lab Simulation Workflow Real-World Deployment Notes Assessment / Exercises 6.3 Curriculum Modules (v1) Module 1 — The Robotic Nervous System (ROS 2) ROS 2 architecture Nodes, Topics, Services, Actions rclpy Python control URDF humanoid modeling Module 2 — The Digital Twin Gazebo & Unity simulations Physics, gravity, collision Sensor simulation Human-robot interaction environments Module 3 — The AI-Robot Brain NVIDIA Isaac Sim Synthetic data generation Isaac ROS VSLAM Nav2 navigation & planning Module 4 — Vision-Language-Action Speech input (Whisper) LLM cognitive planning Natural language → ROS actions Multimodal perception → control loops 7. RAG Chatbot Specification 7.1 Architecture Frontend Embedded chat UI inside Docusaurus "Explain selected text" interaction Backend FastAPI OpenAI Agents SDK / ChatKit Neon Serverless Postgres (metadata, sessions) Qdrant Vector DB (content embeddings) 7.2 RAG Constraints (Non-Negotiable) Answers MUST be grounded in: Retrieved book sections only Every response MUST: Cite chapter + section If answer not found: Respond with "This is not covered in the book yet" 7.3 Capabilities Chapter-level Q&A Section-level explanation Selected-text explanation Step-by-step tutoring Concept → code → simulation linkage 8. AI-Native Authoring Requirements Content written using Spec-Kit Plus Claude Code used for: Drafting chapters Generating exercises Creating reusable subagents Reusable intelligence encouraged: Concept explainer agents Lab walkthrough agents Assessment feedback agents 9. Quality Gates 9.1 Content Quality Clear learning objectives No misleading scientific claims Embodiment relevance verified 9.2 Code Quality Runnable Documented Version-controlled Simulation-tested 9.3 AI Quality No hallucinations Grounded citations Deterministic retrieval paths 10. Governance & Review Maintainers aligned with Panaversity leadership All PRs must: Pass AI + human review Meet constitution principles Amendments: Require version bump Public changelog Migration notes if breaking"

## Clarifications
### Session 2026-02-05
- Q: For the AI-Native Digital Textbook, what authentication and authorization requirements should be implemented for student access? → A: No authentication
- Q: For the AI-Native Digital Textbook, what data privacy and retention policy should be implemented for student data? → A: Simple implementation first, with advanced features added after successful deployment
- Q: For the AI-Native Digital Textbook, which external services should the system depend on for core functionality? → A: Minimal dependencies - only essential services like hosting and basic authentication
- Q: For the AI-Native Digital Textbook, what are the expected performance and scalability targets? → A: Scale dynamically based on demand, with consistent performance
- Q: For the AI-Native Digital Textbook, should the system support offline access to content? → A: Progressive enhancement - basic content available offline, advanced features online
- Q: Which frontend framework should be used for implementing the textbook interface? → A: React with Docusaurus
- Q: Which backend technology should be used for implementing the RAG chatbot functionality? → A: Python with FastAPI
- Q: Which vector database should be used for storing and retrieving content embeddings for the RAG system? → A: Qdrant
- Q: Which simulation environment should be used for the robotics codebase and hands-on labs? → A: ROS 2 with Gazebo
- Q: Which database should be used for storing student sessions, metadata, and other structured data? → A: PostgreSQL
- Q: How should the AI-Native Digital Textbook be deployed? → A: Serverless deployment entirely on Vercel
- Q: How should the textbook content be managed and updated? → A: Markdown files in Git repository with version control
- Q: Which AI model provider should be used for the RAG system? → A: Self-hosted open-source models (e.g., Llama)
- Q: Which monitoring and analytics solution should be used for the system? → A: Simple implementation first, with advanced features added after successful deployment
- Q: What testing strategy should be employed for the AI-Native Digital Textbook? → A: Jest for frontend, PyTest for backend
- Q: How should the system handle errors when the AI tutor cannot generate a response? → A: Provide a fallback response with links to relevant textbook sections

## Tech Stack & Implementation Approach

The following technologies have been selected for implementing the AI-Native Digital Textbook:

- **Frontend**: React with Docusaurus for the textbook interface
- **Backend**: Python with FastAPI for the RAG chatbot functionality
- **Vector Database**: Qdrant for storing and retrieving content embeddings
- **Simulation Environment**: ROS 2 with Gazebo for robotics codebase and hands-on labs
- **Database**: PostgreSQL for storing student sessions, metadata, and other structured data
- **Deployment**: Serverless deployment entirely on Vercel
- **Content Management**: Markdown files in Git repository with version control
- **AI Model Provider**: Self-hosted open-source models (e.g., Llama)
- **Testing Strategy**: Jest for frontend, PyTest for backend

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Access Interactive Learning Content (Priority: P1)

A student accesses the AI-native digital textbook to learn about Physical AI and Humanoid Robotics. They navigate through modules, chapters, and sections to study concepts, view diagrams, and read explanations. The content is structured in a machine-readable format that allows for easy navigation and searchability.

**Why this priority**: This is the foundational user experience that enables all other interactions with the textbook. Without accessible content, the AI tutor and other features cannot function effectively.

**Independent Test**: Can be fully tested by navigating through the textbook interface and verifying that content is displayed correctly, with proper learning objectives, explanations, and visual aids.

**Acceptance Scenarios**:

1. **Given** a student opens the textbook website, **When** they click on a module/chapter/section, **Then** the content loads with learning objectives, conceptual explanations, and visual aids
2. **Given** a student is viewing content on a mobile device or low-bandwidth connection, **When** they navigate through the textbook, **Then** the content loads quickly and remains accessible
3. **Given** a student wants to search for specific content, **When** they use the search function, **Then** relevant results from the textbook appear based on their query

---

### User Story 2 - Engage with AI Tutor for Concept Clarification (Priority: P1)

A student encounters a concept they don't understand and uses the integrated RAG chatbot to ask questions about the textbook content. The AI tutor responds with explanations grounded in the book's content, citing specific chapters and sections.

**Why this priority**: This is the core differentiator of the AI-native textbook - providing an intelligent tutoring system that understands the material better than a human TA.

**Independent Test**: Can be fully tested by asking various questions about the textbook content and verifying that responses are accurate, grounded in the book, and include proper citations.

**Acceptance Scenarios**:

1. **Given** a student selects text in the textbook, **When** they ask the AI tutor to explain it, **Then** the tutor provides a contextual explanation citing the relevant section
2. **Given** a student asks a question about content covered in the book, **When** they submit the query to the chatbot, **Then** the response includes accurate information with chapter and section citations
3. **Given** a student asks about content not covered in the book, **When** they submit the query, **Then** the system responds with "This is not covered in the book yet"

---

### User Story 3 - Execute Hands-On Labs and Simulations (Priority: P2)

A student follows along with hands-on labs and simulation exercises provided in the textbook. They access runnable code examples and simulation environments to practice implementing the concepts they've learned.

**Why this priority**: This bridges the gap between theoretical knowledge and practical application, allowing students to see how concepts work in practice.

**Independent Test**: Can be fully tested by running the provided code examples and simulations to verify they execute correctly and demonstrate the intended concepts.

**Acceptance Scenarios**:

1. **Given** a student accesses a lab exercise in the textbook, **When** they run the provided code, **Then** the simulation executes successfully and demonstrates the concept
2. **Given** a student modifies parameters in a simulation, **When** they run it again, **Then** they can observe how changes affect the robot's behavior
3. **Given** a student completes a lab exercise, **When** they submit their results, **Then** they receive feedback on their implementation

---

### User Story 4 - Transition Between Digital AI and Physical World Concepts (Priority: P3)

A learner explores how digital AI concepts connect to physical embodiment in robotics, understanding the relationship between perception, reasoning, action, and human interaction in Physical AI systems.

**Why this priority**: This addresses the core problem statement of bridging the gap between software intelligence and physical embodiment.

**Independent Test**: Can be fully tested by verifying that the curriculum content clearly demonstrates connections between digital concepts and their physical implementations.

**Acceptance Scenarios**:

1. **Given** a student reads about an AI concept in the textbook, **When** they look for its physical implementation, **Then** they find clear explanations of how it applies to robotics
2. **Given** a student learns about a robotic system, **When** they explore the underlying AI, **Then** they understand how cognition drives physical behavior

---

### Edge Cases

- What happens when a student asks the AI tutor about content that exists in multiple sections of the book?
- How does the system handle queries that span multiple chapters or modules?
- What occurs when the AI tutor encounters ambiguous questions that could relate to multiple topics?
- How does the system respond when the simulation environment is temporarily unavailable?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a web-based interface for accessing the digital textbook content
- **FR-002**: System MUST allow students to navigate content organized as modules → chapters → sections
- **FR-003**: System MUST provide full-text search capabilities across all textbook content
- **FR-004**: System MUST include an AI-powered chatbot that answers questions based solely on textbook content
- **FR-005**: System MUST cite specific chapters and sections when providing answers through the chatbot
- **FR-006**: System MUST respond with "This is not covered in the book yet" when questions cannot be answered from the textbook
- **FR-007**: System MUST host runnable code examples and simulation exercises
- **FR-008**: System MUST be accessible on mobile devices and low-bandwidth connections
- **FR-009**: System MUST provide learning objectives for each module and chapter
- **FR-010**: System MUST include visual aids such as diagrams and flows for each concept
- **FR-011**: System MUST offer assessment and exercise components for each module
- **FR-012**: System MUST support "explain selected text" functionality for contextual tutoring
- **FR-013**: System MUST maintain versioning tied to git tags for textbook content
- **FR-014**: System MUST provide step-by-step tutoring capabilities for complex concepts
- **FR-015**: System MUST link concepts to corresponding code and simulation examples
- **FR-016**: System MUST support progressive enhancement with basic content available offline and advanced features online
- **FR-017**: System MUST provide a fallback response with links to relevant textbook sections when the AI tutor cannot generate a response

### Key Entities

- **Textbook Content**: Represents the educational material organized in modules, chapters, and sections with learning objectives, explanations, and visual aids
- **Student Session**: Represents a user's interaction with the textbook, including their progress, queries to the AI tutor, and completed assessments
- **AI Tutor Response**: Represents the AI-generated explanations and answers based on textbook content with proper citations
- **Simulation Exercise**: Represents runnable code examples and simulation environments that demonstrate textbook concepts
- **Assessment Item**: Represents questions and exercises that test student understanding of the material

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can navigate and access textbook content within 3 clicks from any page
- **SC-002**: The AI tutor provides accurate answers with proper citations 95% of the time
- **SC-003**: At least 90% of students successfully complete the first hands-on lab exercise
- **SC-004**: Students can find relevant content through search functionality within 30 seconds
- **SC-005**: The system achieves 99% uptime during peak academic hours
- **SC-006**: Students report a 40% improvement in understanding Physical AI concepts compared to traditional textbooks
- **SC-007**: The AI tutor reduces the average time to clarify concepts by 50% compared to traditional resources
- **SC-008**: All simulation exercises run successfully without modification in 95% of attempts
- **SC-009**: System scales dynamically based on demand while maintaining consistent performance

---

## 11. Detailed Implementation Requirements

### 11.1 RAG System Implementation (P0 - BLOCKING)

#### 11.1.1 Qdrant Vector Database Setup
- **FR-018**: System MUST deploy Qdrant vector database (cloud-hosted or self-hosted)
- **FR-019**: System MUST configure Qdrant collections for textbook content embeddings
- **FR-020**: System MUST use embedding dimension of 1536 (OpenAI compatible) or 1024 (self-hosted models)
- **FR-021**: System MUST index each textbook section as a separate vector with metadata (chapter, section, module, git SHA)
- **FR-022**: System MUST support incremental re-indexing when content changes
- **Technical Requirements**:
  - Collection name: `textbook_content`
  - Distance metric: Cosine similarity
  - Payload schema: `{module: string, chapter: string, section: string, content: string, git_sha: string, created_at: timestamp}`

#### 11.1.2 Text Embedding Pipeline
- **FR-023**: System MUST parse all Markdown files from `frontend/docs/` directory
- **FR-024**: System MUST split content into chunks of 500-1000 tokens with 50-token overlap
- **FR-025**: System MUST generate embeddings using self-hosted Llama model or OpenAI-compatible API
- **FR-026**: System MUST store embeddings in Qdrant with full metadata for citation
- **FR-027**: System MUST provide CLI command to rebuild all embeddings: `python backend/scripts/rebuild_embeddings.py`
- **FR-028**: System MUST detect content changes via git hooks and trigger re-indexing

#### 11.1.3 Retrieval Logic for Chatbot Queries
- **FR-029**: System MUST convert user queries to embeddings using same model as content
- **FR-030**: System MUST perform top-k retrieval (k=5) from Qdrant with similarity threshold 0.7
- **FR-031**: System MUST re-rank retrieved results using cross-encoder or MMR (Maximal Marginal Relevance)
- **FR-032**: System MUST concatenate top results into context window (max 4000 tokens)
- **FR-033**: System MUST filter results by module/chapter if user specifies context
- **FR-034**: System MUST log all retrieval queries for analytics and improvement

#### 11.1.4 LLM Integration for Response Generation
- **FR-035**: System MUST integrate self-hosted Llama model (7B or 13B) via Ollama, vLLM, or TGI
- **FR-036**: System MUST construct prompt with: system instructions, retrieved context, user query
- **FR-037**: System MUST enforce response grounding: "Answer ONLY from provided context"
- **FR-038**: System MUST generate responses with max 500 tokens for clarity
- **FR-039**: System MUST extract citation metadata from retrieved chunks for attribution
- **FR-040**: System MUST handle LLM errors gracefully with fallback responses
- **Prompt Template**:
  ```
  You are an AI tutor for Physical AI & Humanoid Robotics. Answer ONLY from the context below.
  
  CONTEXT:
  {retrieved_chunks}
  
  QUESTION: {user_query}
  
  INSTRUCTIONS:
  - Answer based solely on the context
  - Cite chapter and section for each fact
  - If context doesn't contain the answer, say "This is not covered in the book yet"
  - Use clear, educational language
  ```

#### 11.1.5 Citation Extraction
- **FR-041**: System MUST extract module, chapter, and section from retrieved chunk metadata
- **FR-042**: System MUST format citations as: `Module X: Chapter Y - Section Z`
- **FR-043**: System MUST display citation count (e.g., "3 sources cited")
- **FR-044**: System MUST link citations to textbook sections (clickable)
- **FR-045**: System MUST show confidence score based on retrieval similarity (0.0-1.0)

#### 11.1.6 "Explain Selected Text" Feature
- **FR-046**: System MUST capture user-selected text from frontend (highlight)
- **FR-047**: System MUST send selected text + surrounding context (200 tokens) to backend
- **FR-048**: System MUST retrieve related content from Qdrant using selected text embedding
- **FR-049**: System MUST generate explanation connecting selected text to broader concepts
- **FR-050**: System MUST display explanation in popup/overlay near selected text

---

### 11.2 AI Tutor Chatbot UI (P0 - BLOCKING)

#### 11.2.1 Chat Widget Embedding
- **FR-051**: System MUST embed chat widget as floating button in bottom-right corner of Docusaurus
- **FR-052**: System MUST provide chat window with: message history, input field, send button
- **FR-053**: System MUST style chat widget to match Docusaurus theme (light/dark mode)
- **FR-054**: System MUST make chat widget collapsible (minimize/maximize)
- **FR-055**: System MUST persist chat history in localStorage (session-based)
- **UI Components**:
  - Floating action button (FAB) with chat icon
  - Chat window: 400px width, 600px height (responsive)
  - Message bubbles: user (right, blue), assistant (left, gray)
  - Input field with character limit (2000 chars)
  - Loading indicator during API calls

#### 11.2.2 "Explain Selected Text" Interaction
- **FR-056**: System MUST detect text selection in textbook content area
- **FR-057**: System MUST show popup button "Explain this" near selected text
- **FR-058**: System MUST open chat widget with pre-filled query: "Explain: {selected_text}"
- **FR-059**: System MUST highlight selected text in chat context
- **FR-060**: System MUST dismiss selection popup on click outside

#### 11.2.3 Citation Display
- **FR-061**: System MUST display citations below each AI response
- **FR-062**: System MUST format citations as clickable links to textbook sections
- **FR-063**: System MUST show confidence indicator (e.g., star rating or percentage)
- **FR-064**: System MUST allow expanding citations to show retrieved context snippets

#### 11.2.4 Fallback Response Handling
- **FR-065**: System MUST display friendly fallback: "This is not covered in the book yet"
- **FR-066**: System MUST suggest related topics that ARE covered (based on query keywords)
- **FR-067**: System MUST provide link to table of contents for manual navigation
- **FR-068**: System MUST log fallback queries for content gap analysis

#### 11.2.5 Step-by-Step Tutoring Mode
- **FR-069**: System MUST support multi-turn conversations with context retention
- **FR-070**: System MUST break complex explanations into numbered steps
- **FR-071**: System MUST offer "Next step" and "Previous step" navigation in explanations
- **FR-072**: System MUST provide "I don't understand" button to simplify explanation
- **FR-073**: System MUST track conversation state in session (PostgreSQL or localStorage)

---

### 11.3 Search Functionality (P1)

#### 11.3.1 Full-Text Search
- **FR-074**: System MUST index all textbook Markdown content for full-text search
- **FR-075**: System MUST support keyword search with highlighting of matched terms
- **FR-076**: System MUST display search results with: title, snippet, relevance score
- **FR-077**: System MUST support search filters: by module, chapter, content type
- **FR-078**: System MUST implement search-as-you-type with 300ms debounce
- **Technical Implementation**:
  - Use FlexSearch or MiniSearch for client-side search
  - Index structure: `{id, title, content, module, chapter, section, url}`
  - Result limit: 20 results per query

#### 11.3.2 AI-Powered Semantic Search
- **FR-079**: System MUST use same embeddings as RAG for semantic search
- **FR-080**: System MUST combine keyword + semantic search results (hybrid search)
- **FR-081**: System MUST re-rank results using BM25 + vector similarity
- **FR-082**: System MUST display "semantic match" indicator for non-keyword matches
- **FR-083**: System MUST support natural language queries (e.g., "How do I control a robot with Python?")

#### 11.3.3 Search Results Display
- **FR-084**: System MUST show search results in overlay/dropdown below search bar
- **FR-085**: System MUST group results by module/chapter
- **FR-086**: System MUST highlight matched keywords in result snippets
- **FR-087**: System MUST show result count (e.g., "15 results found")
- **FR-088**: System MUST support keyboard navigation (arrow keys, Enter to select)
- **FR-089**: System MUST maintain search history (last 10 queries) in localStorage

---

### 11.4 Content Expansion (P2)

#### 11.4.1 Detailed Chapter Explanations
- **FR-090**: System MUST expand each chapter to minimum 2000 words of explanatory content
- **FR-091**: System MUST include "Key Takeaways" section at end of each chapter
- **FR-092**: System MUST provide "Prerequisites" section linking to prerequisite chapters
- **FR-093**: System MUST include "Common Misconceptions" callout boxes
- **FR-094**: System MUST add "Real-World Applications" examples for each concept

#### 11.4.2 Interactive Diagrams
- **FR-095**: System MUST convert static diagrams to interactive Mermaid.js or D3.js visualizations
- **FR-096**: System MUST allow users to click diagram elements for detailed explanations
- **FR-097**: System MUST provide zoom/pan controls for complex diagrams
- **FR-098**: System MUST animate data flow in architecture diagrams
- **FR-099**: System MUST ensure diagrams are accessible (alt text, keyboard navigation)

#### 11.4.3 Real-World Deployment Notes
- **FR-100**: System MUST include "Deployment Checklist" for each hands-on lab
- **FR-101**: System MUST provide troubleshooting guides for common errors
- **FR-102**: System MUST document hardware requirements for real-robot deployment
- **FR-103**: System MUST include safety warnings for physical robot operations
- **FR-104**: System MUST link to manufacturer documentation for hardware components

#### 11.4.4 Glossary and Index
- **FR-105**: System MUST maintain glossary of all technical terms (minimum 200 terms)
- **FR-106**: System MUST link glossary terms inline (hover definition popup)
- **FR-107**: System MUST provide alphabetical index page with links to all concepts
- **FR-108**: System MUST support glossary search with fuzzy matching
- **FR-109**: System MUST auto-generate glossary from content using NLP extraction

---

### 11.5 Advanced Features (P3)

#### 11.5.1 Offline Content Access
- **FR-110**: System MUST implement Service Worker for offline caching
- **FR-111**: System MUST cache all textbook Markdown content on first visit
- **FR-112**: System MUST show offline indicator when network is unavailable
- **FR-113**: System MUST queue chat queries for sync when back online
- **FR-114**: System MUST limit offline mode to text content (no AI features)

#### 11.5.2 Multi-Language Support
- **FR-115**: System MUST support i18n framework (react-i18next or Docusaurus i18n)
- **FR-116**: System MUST provide language switcher in navbar
- **FR-117**: System MUST support Urdu and English as initial languages
- **FR-118**: System MUST store translations in separate Markdown files per locale
- **FR-119**: System MUST auto-detect user language from browser settings

#### 11.5.3 Community Contribution Workflow
- **FR-120**: System MUST provide "Suggest Edit" button on each chapter
- **FR-121**: System MUST open GitHub issue with pre-filled template for content suggestions
- **FR-122**: System MUST link to GitHub PR workflow for advanced contributors
- **FR-123**: System MUST display contributor acknowledgments on each chapter
- **FR-124**: System MUST implement content review workflow (AI + human review)

#### 11.5.4 Versioning System
- **FR-125**: System MUST display current textbook version (from git tag) in footer
- **FR-126**: System MUST provide version selector to view older textbook versions
- **FR-127**: System MUST show "What's New" changelog for each version
- **FR-128**: System MUST warn users if viewing outdated version
- **FR-129**: System MUST auto-redirect to latest stable version by default

---

### 11.6 Testing & Validation (P1)

#### 11.6.1 Comprehensive Test Suites
- **FR-130**: System MUST achieve 80%+ code coverage for backend (pytest-cov)
- **FR-131**: System MUST achieve 80%+ code coverage for frontend (Jest)
- **FR-132**: System MUST include unit tests for all API endpoints
- **FR-133**: System MUST include integration tests for RAG pipeline
- **FR-134**: System MUST include E2E tests for critical user journeys (Playwright/Cypress)
- **FR-135**: System MUST include contract tests for API schemas
- **FR-136**: System MUST run tests on CI/CD pipeline (GitHub Actions)

#### 11.6.2 Citation Accuracy
- **FR-137**: System MUST achieve 95%+ citation accuracy (manual evaluation on test set)
- **FR-138**: System MUST log all citations for audit and quality analysis
- **FR-139**: System MUST provide "Report Incorrect Citation" feedback button
- **FR-140**: System MUST run weekly evaluation on golden test dataset

#### 11.6.3 Mobile & Low-Bandwidth Testing
- **FR-141**: System MUST pass Lighthouse accessibility score >90
- **FR-142**: System MUST load initial page in <3 seconds on 3G network
- **FR-143**: System MUST support screen readers (WCAG 2.1 AA compliance)
- **FR-144**: System MUST test on iOS Safari, Android Chrome, Firefox Mobile
- **FR-145**: System MUST provide low-bandwidth mode (disable images, keep text)

#### 11.6.4 Load Testing
- **FR-146**: System MUST handle 1000 concurrent users without degradation
- **FR-147**: System MUST maintain p95 latency <500ms under load
- **FR-148**: System MUST auto-scale Vercel serverless functions based on demand
- **FR-149**: System MUST implement rate limiting (100 requests/minute per user)
- **FR-150**: System MUST provide load testing scripts (locust.io or k6)
- **Performance Budgets**:
  - Time to First Byte (TTFB): <200ms
  - First Contentful Paint (FCP): <1.5s
  - Time to Interactive (TTI): <3.5s
  - API p95 latency: <500ms
  - Error rate: <0.1%

---

## 12. Implementation Roadmap

### Phase 1: Foundation (Weeks 1-2) ✅ COMPLETE
- [x] Project setup (Docusaurus, FastAPI, Vercel)
- [x] Constitution and specification
- [x] Basic textbook structure (Modules 1-4)
- [x] Backend API skeleton

### Phase 2: RAG System (Weeks 3-4) 🔴 IN PROGRESS
- [ ] Qdrant setup and embedding pipeline
- [ ] LLM integration (self-hosted or OpenAI-compatible)
- [ ] Citation extraction and display
- [ ] Backend retrieval logic

### Phase 3: Chatbot UI (Weeks 5-6) 🔴 NOT STARTED
- [ ] Chat widget component
- [ ] "Explain selected text" interaction
- [ ] Citation UI and fallback handling
- [ ] Step-by-step tutoring mode

### Phase 4: Search (Week 7) 🔴 NOT STARTED
- [ ] Full-text search implementation
- [ ] Semantic search integration
- [ ] Search results UI

### Phase 5: Content Polish (Weeks 8-9) 🔴 NOT STARTED
- [ ] Detailed chapter expansions
- [ ] Interactive diagrams
- [ ] Glossary and index
- [ ] Real-world deployment notes

### Phase 6: Advanced Features (Weeks 10-11) 🔴 NOT STARTED
- [ ] Offline support
- [ ] Multi-language (Urdu + English)
- [ ] Community contributions
- [ ] Versioning system

### Phase 7: Testing & Launch (Week 12) 🔴 NOT STARTED
- [ ] Comprehensive test suites
- [ ] Load testing
- [ ] Mobile/accessibility testing
- [ ] Performance optimization
- [ ] Public launch