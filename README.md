# AI-Native Digital Textbook on Physical AI & Humanoid Robotics

[![Deploy to Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/agent-book-factory/agent-book-factory)
[![Tests](https://img.shields.io/badge/tests-pytest%20%2B%20jest-blue)](https://github.com/agent-book-factory/agent-book-factory)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

The world's first **AI-native digital textbook** for Physical AI & Humanoid Robotics, where the book is machine-readable and an AI tutor understands the content better than any human TA.

## 🤖 AI Tutor Features

- **Context-aware tutoring** - Answers based solely on textbook content
- **Selected text explanation** - Highlight any text and ask for explanation
- **Step-by-step guidance** - Complex concepts broken down
- **Citations** - Every response cites exact chapter and section

## 🚀 Quick Start

### Prerequisites

- **Node.js 18+** and npm
- **Python 3.11+**
- **Docker & Docker Compose** (for local databases)
- **Vercel account** (for deployment)

### 1. Clone the Repository

```bash
git clone https://github.com/agent-book-factory/agent-book-factory.git
cd agent-book-factory
```

### 2. Start Infrastructure (Docker)

```bash
# Start PostgreSQL and Qdrant
docker-compose up -d
```

This starts:
- **PostgreSQL** on `localhost:5432`
- **Qdrant** on `localhost:6333`
- **pgAdmin** on `localhost:5050` (optional)

### 3. Backend Setup (FastAPI)

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Copy environment configuration
cp .env.example .env

# Edit .env and configure:
# - DATABASE_URL (default: postgresql://agentbook:agentbook_secret@localhost:5432/agent_book_db)
# - QDRANT_URL (default: http://localhost:6333)
# - OPENAI_API_KEY (your API key)

# Initialize Qdrant collection and index content
python scripts/rebuild_embeddings.py

# Start the server
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Visit `http://localhost:8000/docs` for API documentation.

### 4. Frontend Setup (Docusaurus)

```bash
cd frontend

# Install dependencies
npm install

# Copy environment configuration
cp .env.example .env

# Edit .env and configure:
# - REACT_APP_API_URL (default: http://localhost:8000)

# Start development server
npm run start
```

Visit `http://localhost:3000`

---

## 📚 Curriculum Structure

### Module 1: The Robotic Nervous System (ROS 2)
- ROS 2 architecture
- Nodes, Topics, Services, Actions
- Python control with rclpy
- URDF humanoid modeling

### Module 2: The Digital Twin
- Gazebo & Unity simulations
- Physics, gravity, collision
- Sensor simulation
- Human-robot interaction environments

### Module 3: The AI-Robot Brain
- NVIDIA Isaac Sim
- Synthetic data generation
- Isaac ROS VSLAM
- Nav2 navigation & planning

### Module 4: Vision-Language-Action
- Speech input (Whisper)
- LLM cognitive planning
- Natural language → ROS actions
- Multimodal perception → control loops

## 📚 Curriculum Structure

### Module 1: The Robotic Nervous System (ROS 2)
- ROS 2 architecture
- Nodes, Topics, Services, Actions
- Python control with rclpy
- URDF humanoid modeling

### Module 2: The Digital Twin
- Gazebo & Unity simulations
- Physics, gravity, collision
- Sensor simulation
- Human-robot interaction environments

### Module 3: The AI-Robot Brain
- NVIDIA Isaac Sim
- Synthetic data generation
- Isaac ROS VSLAM
- Nav2 navigation & planning

### Module 4: Vision-Language-Action
- Speech input (Whisper)
- LLM cognitive planning
- Natural language → ROS actions
- Multimodal perception → control loops

---

## 🧪 Testing

### Backend Tests (PyTest)

```bash
cd backend
source venv/bin/activate  # Windows: venv\Scripts\activate

# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test category
pytest tests/unit -v
pytest tests/integration -v
pytest tests/contract -v
```

### Frontend Tests (Jest)

```bash
cd frontend

# Run all tests
npm run test

# Run in watch mode
npm run test:watch

# Run with coverage
npm run test:coverage
```

## 📦 Deployment to Vercel

### One-Click Deploy

Click the "Deploy to Vercel" button above, or:

### Manual Deploy

1. **Install Vercel CLI**
   ```bash
   npm install -g vercel
   ```

2. **Login to Vercel**
   ```bash
   vercel login
   ```

3. **Deploy**
   ```bash
   vercel
   ```

4. **Production Deploy**
   ```bash
   vercel --prod
   ```

### Environment Variables

Configure these in Vercel dashboard:

```env
# Backend (Python)
DATABASE_URL=your-postgres-url
QDRANT_URL=your-qdrant-url
OPENAI_API_KEY=your-api-key

# Frontend (Node.js)
REACT_APP_API_URL=https://your-api.vercel.app
```

## 🛠️ Tech Stack

| Component | Technology |
|-----------|-----------|
| Frontend | React + Docusaurus |
| Backend | FastAPI (Python) |
| Deployment | Vercel (Serverless) |
| Vector DB | Qdrant (RAG) |
| Database | PostgreSQL (Neon) |
| Testing | Jest + PyTest |
| Embeddings | sentence-transformers |
| LLM | OpenAI-compatible API |

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Vercel Platform                   │
├─────────────────────────────────────────────────────┤
│  ┌──────────────────┐    ┌─────────────────────┐   │
│  │   Docusaurus     │    │   FastAPI Serverless│   │
│  │   (Static Site)  │    │   (/api/* routes)   │   │
│  │                  │    │                     │   │
│  │  - Textbook UI   │    │  - Chat API         │   │
│  │  - Navigation    │    │  - Search API       │   │
│  │  - Search UI     │    │  - RAG Processing   │   │
│  └──────────────────┘    └─────────────────────┘   │
│                              │                      │
│                              ↓                      │
│                    ┌─────────────────┐             │
│                    │  External Services│            │
│                    │  - Qdrant (Vector)│            │
│                    │  - Neon (Postgres)│            │
│                    └─────────────────┘             │
└─────────────────────────────────────────────────────┘
```

## 📖 Development Workflow

### Git Workflow Agent

This project includes an autonomous Git workflow agent:

```bash
# Run the Git workflow agent
python git_workflow_agent.py
```

It automatically:
- Analyzes repository state
- Creates meaningful branch names
- Generates conventional commit messages
- Creates pull requests (if `gh` CLI is available)

### Spec-Driven Development

This project follows Spec-Driven Development (SDD) practices:
- All features start with a specification (`specs/`)
- Architecture decisions are documented (`plan.md`)
- Tasks are testable and traceable (`tasks.md`)
- Prompt history is recorded (`history/prompts/`)

## 🎯 Success Criteria

- ✅ Students can navigate content within 3 clicks
- ✅ AI tutor provides accurate, cited answers
- ✅ All simulation exercises are runnable
- ✅ Mobile and low-bandwidth accessible
- ✅ 99% uptime during peak hours

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Make your changes
4. Run tests (`pytest` and `npm test`)
5. Submit a PR

## 📞 Support

- **Documentation**: See `/frontend/docs` for textbook content
- **API Documentation**: `http://localhost:8000/docs` when running backend
- **GitHub Issues**: Report bugs and feature requests
- **Constitution**: See `.specify/memory/constitution.md` for project principles

## 📝 License

Open content - see LICENSE file

---

**Built with ❤️ for Panaversity**
