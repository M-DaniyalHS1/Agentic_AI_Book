# Data Model: AI-Native Digital Textbook on Physical AI & Humanoid Robotics

## Entities

### Textbook Content
- **Fields**:
  - id (string): Unique identifier for the content module
  - title (string): Title of the module/chapter/section
  - type (enum): 'module', 'chapter', or 'section'
  - content (string): Markdown content of the section
  - learning_objectives (array): List of learning objectives
  - visual_aids (array): Links to diagrams, images, videos
  - hands_on_labs (array): Associated lab exercises
  - assessments (array): Associated quizzes/exercises
  - parent_id (string): Reference to parent entity (null for modules)
  - children_ids (array): References to child entities
  - created_at (datetime): Timestamp of creation
  - updated_at (datetime): Timestamp of last update
  - version (string): Git tag associated with this version
- **Relationships**: Hierarchical structure (modules contain chapters, chapters contain sections)
- **Validation**: Content must be in valid markdown format, learning objectives must be non-empty for modules and chapters

### Student Session
- **Fields**:
  - id (string): Unique identifier for the session
  - student_id (string): Identifier for the student (anonymous for now, future enhancement)
  - current_location (object): Current position in the textbook (module, chapter, section)
  - progress (object): Map of completed content with timestamps
  - ai_queries (array): List of questions asked to the AI tutor
  - ai_responses (array): List of responses from the AI tutor
  - lab_completions (array): Record of completed lab exercises
  - assessment_scores (array): Scores on assessments
  - created_at (datetime): Session start time
  - updated_at (datetime): Last activity timestamp
- **Relationships**: References to Textbook Content entities
- **Validation**: Session must have a valid current location in the textbook

### AI Tutor Response
- **Fields**:
  - id (string): Unique identifier for the response
  - query (string): Original question from the student
  - response (string): AI-generated answer
  - cited_sections (array): IDs of textbook sections used to generate the response
  - confidence_score (float): Confidence level in the response (0-1)
  - generated_at (datetime): Timestamp of response generation
  - fallback_triggered (boolean): Whether a fallback response was used
- **Relationships**: References to Textbook Content entities that were cited
- **Validation**: Response must cite at least one section from the textbook or indicate that the topic is not covered

### Simulation Exercise
- **Fields**:
  - id (string): Unique identifier for the exercise
  - title (string): Title of the exercise
  - description (string): Description of the exercise
  - module_id (string): Reference to the associated module
  - code_snippets (array): Associated code examples
  - simulation_config (object): Configuration for the simulation environment
  - learning_outcomes (array): Expected learning outcomes
  - prerequisites (array): Prerequisites for the exercise
  - created_at (datetime): Timestamp of creation
  - updated_at (datetime): Timestamp of last update
- **Relationships**: References to Textbook Content entities
- **Validation**: Must have valid simulation configuration and executable code snippets

### Assessment Item
- **Fields**:
  - id (string): Unique identifier for the assessment
  - title (string): Title of the assessment
  - type (enum): 'quiz', 'exercise', 'project', 'exam'
  - content (string): Question content in markdown
  - options (array): For multiple choice questions
  - correct_answer (string): Correct answer
  - explanation (string): Explanation of the correct answer
  - associated_content (array): IDs of textbook sections this assesses
  - difficulty_level (enum): 'beginner', 'intermediate', 'advanced'
  - created_at (datetime): Timestamp of creation
  - updated_at (datetime): Timestamp of last update
- **Relationships**: References to Textbook Content entities
- **Validation**: Must have a correct answer and explanation for all assessment types