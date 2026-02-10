# API Contracts: AI-Native Digital Textbook on Physical AI & Humanoid Robotics

## 1. Textbook Content API

### GET /api/content/{module_id}/{chapter_id}/{section_id}
**Description**: Retrieve specific textbook content by module, chapter, and section IDs
**Parameters**:
- module_id (path): ID of the module
- chapter_id (path): ID of the chapter
- section_id (path): ID of the section
**Response**:
- 200 OK: Returns the content object with title, content, learning objectives, visual aids, etc.
- 404 Not Found: If the content doesn't exist

### GET /api/modules
**Description**: Retrieve a list of all modules
**Response**:
- 200 OK: Returns an array of module objects with titles and descriptions

### GET /api/modules/{module_id}/chapters
**Description**: Retrieve chapters for a specific module
**Parameters**:
- module_id (path): ID of the module
**Response**:
- 200 OK: Returns an array of chapter objects
- 404 Not Found: If the module doesn't exist

## 2. AI Tutor API

### POST /api/tutor/query
**Description**: Submit a question to the AI tutor
**Request Body**:
```json
{
  "query": "string",
  "context": {
    "module_id": "string",
    "chapter_id": "string",
    "section_id": "string"
  },
  "session_id": "string"
}
```
**Response**:
- 200 OK: Returns the AI response with citations
```json
{
  "response": "string",
  "citations": [
    {
      "module_id": "string",
      "chapter_id": "string",
      "section_id": "string",
      "title": "string"
    }
  ],
  "confidence_score": "float"
}
```
- 400 Bad Request: If the query is malformed
- 500 Internal Server Error: If the AI service fails

### POST /api/tutor/explain-text
**Description**: Ask the AI tutor to explain selected text
**Request Body**:
```json
{
  "selected_text": "string",
  "context": {
    "module_id": "string",
    "chapter_id": "string",
    "section_id": "string"
  },
  "session_id": "string"
}
```
**Response**:
- 200 OK: Returns the explanation with citations
- 400 Bad Request: If the selected_text is empty

## 3. Student Session API

### POST /api/sessions
**Description**: Create a new student session
**Request Body**:
```json
{
  "initial_location": {
    "module_id": "string",
    "chapter_id": "string",
    "section_id": "string"
  }
}
```
**Response**:
- 201 Created: Returns the new session object with session_id

### GET /api/sessions/{session_id}
**Description**: Retrieve a student session
**Parameters**:
- session_id (path): ID of the session
**Response**:
- 200 OK: Returns the session object
- 404 Not Found: If the session doesn't exist

### PUT /api/sessions/{session_id}/progress
**Description**: Update student progress
**Parameters**:
- session_id (path): ID of the session
**Request Body**:
```json
{
  "location": {
    "module_id": "string",
    "chapter_id": "string",
    "section_id": "string"
  },
  "completed": "boolean"
}
```
**Response**:
- 200 OK: Returns updated session object

## 4. Simulation Exercise API

### GET /api/exercises
**Description**: Retrieve all simulation exercises
**Query Parameters**:
- module_id (optional): Filter by module
- difficulty (optional): Filter by difficulty level
**Response**:
- 200 OK: Returns an array of exercise objects

### GET /api/exercises/{exercise_id}
**Description**: Retrieve a specific simulation exercise
**Parameters**:
- exercise_id (path): ID of the exercise
**Response**:
- 200 OK: Returns the exercise object
- 404 Not Found: If the exercise doesn't exist

## 5. Assessment API

### GET /api/assessments/{assessment_id}
**Description**: Retrieve a specific assessment
**Parameters**:
- assessment_id (path): ID of the assessment
**Response**:
- 200 OK: Returns the assessment object
- 404 Not Found: If the assessment doesn't exist

### POST /api/assessments/{assessment_id}/submit
**Description**: Submit answers to an assessment
**Parameters**:
- assessment_id (path): ID of the assessment
**Request Body**:
```json
{
  "session_id": "string",
  "answers": [
    {
      "question_id": "string",
      "answer": "string"
    }
  ]
}
```
**Response**:
- 200 OK: Returns the score and feedback
```json
{
  "score": "float",
  "feedback": "string",
  "correct_answers": "array"
}
```