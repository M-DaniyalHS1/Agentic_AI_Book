# Quickstart Guide: AI-Native Digital Textbook on Physical AI & Humanoid Robotics

## Prerequisites

- Node.js (v18 or higher)
- Python (v3.11 or higher)
- ROS 2 (Humble Hawksbill or later)
- Docker and Docker Compose
- Git

## Setting Up the Development Environment

### 1. Clone the Repository

```bash
git clone https://github.com/M-DaniyalHS1/Agentic_AI_Book.git
cd Agentic_AI_Book
```

### 2. Install Dependencies

#### Frontend (Docusaurus)

```bash
cd frontend  # Assuming frontend code will be here after initial setup
npm install
```

#### Backend (FastAPI)

```bash
cd backend  # Assuming backend code will be here after initial setup
pip install -r requirements.txt
```

### 3. Set Up Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
DATABASE_URL=postgresql://username:password@localhost:5432/textbook_db
QDRANT_URL=http://localhost:6333
OPENAI_API_KEY=your_openai_api_key_here  # If using OpenAI models
LLAMA_MODEL_PATH=/path/to/local/llama/model  # If using local models
DEBUG=true
```

### 4. Start Required Services

Using Docker Compose:

```bash
docker-compose up -d postgres qdrant
```

This will start PostgreSQL and Qdrant services needed for the application.

### 5. Initialize the Database

```bash
cd backend
python -m alembic upgrade head
```

### 6. Populate the Vector Database

```bash
cd backend
python -m scripts.populate_embeddings
```

This script will process the textbook content and store embeddings in Qdrant.

## Running the Application

### 1. Start the Backend Server

```bash
cd backend
uvicorn main:app --reload --port 8000
```

The backend API will be available at `http://localhost:8000`.

### 2. Start the Frontend Server

```bash
cd frontend
npm run start
```

The frontend will be available at `http://localhost:3000`.

### 3. Access the Application

Open your browser and navigate to `http://localhost:3000` to access the textbook.

## Running Simulations

### 1. Set Up ROS 2 Environment

Make sure ROS 2 is sourced in your terminal:

```bash
source /opt/ros/humble/setup.bash  # Adjust for your ROS 2 installation
source ./ros_ws/install/setup.bash  # If you have a custom workspace
```

### 2. Launch Simulation Environments

For each module, there will be specific launch files:

```bash
# Example for Module 2 - Digital Twin
cd ros_ws
ros2 launch digital_twin_module.launch.py

# Example for Module 3 - AI-Robot Brain
ros2 launch ai_robot_brain_module.launch.py
```

## Development Workflow

### Adding New Content

1. Create new markdown files in the `content/` directory following the module/chapter/section structure
2. Update the sidebar configuration in `frontend/sidebars.js`
3. Run the embedding script to update the vector database:

```bash
cd backend
python -m scripts.populate_embeddings --update-new-content
```

### Modifying the AI Tutor Behavior

1. Update the prompt templates in `backend/prompts/`
2. Modify the RAG logic in `backend/src/rag_service.py`
3. Test the changes by querying the API directly or through the frontend

### Running Tests

#### Backend Tests

```bash
cd backend
pytest
```

#### Frontend Tests

```bash
cd frontend
npm run test
```

## Troubleshooting

### Common Issues

1. **Port Already in Use**: If ports 8000 or 3000 are already in use, change them in the respective configuration files.

2. **Missing Dependencies**: Make sure all prerequisites are installed and accessible in your PATH.

3. **Database Connection Issues**: Verify that PostgreSQL is running and credentials in `.env` are correct.

4. **Embedding Generation Failure**: Ensure Qdrant is running and accessible at the configured URL.

### Useful Commands

- **Reset Database**: `python -m alembic downgrade base && python -m alembic upgrade head`
- **Rebuild Embeddings**: `python -m scripts.populate_embeddings --force-rebuild`
- **Check Service Status**: `docker-compose ps`

## Next Steps

1. Explore the textbook content through the frontend
2. Try asking questions to the AI tutor
3. Run the simulation exercises for each module
4. Contribute to the content by following the contribution guidelines