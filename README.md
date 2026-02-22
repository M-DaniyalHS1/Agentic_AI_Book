# AI-Native Digital Textbook on Physical AI & Humanoid Robotics

[![Deploy to Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/agent-book-factory/agent-book-factory)

The world's first **AI-native digital textbook** for Physical AI & Humanoid Robotics, where the book is machine-readable and an AI tutor understands the content better than any human TA.

## 🚀 Quick Start

### Prerequisites

- Node.js 18+ and npm
- Python 3.11+
- Vercel account (for deployment)

### Local Development

#### Frontend (Docusaurus)

```bash
cd frontend
npm install
npm run start
```

Visit `http://localhost:3000`

#### Backend (FastAPI)

```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn main:app --reload
```

Visit `http://localhost:8000/docs` for API documentation

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

## 🤖 AI Tutor Features

- **Context-aware tutoring** - Answers based solely on textbook content
- **Selected text explanation** - Highlight any text and ask for explanation
- **Step-by-step guidance** - Complex concepts broken down
- **Citations** - Every response cites exact chapter and section

## 🛠️ Tech Stack

| Component | Technology |
|-----------|-----------|
| Frontend | React + Docusaurus |
| Backend | FastAPI (Python) |
| Deployment | Vercel (Serverless) |
| Vector DB | Qdrant (RAG) |
| Database | PostgreSQL (Neon) |
| Testing | Jest + PyTest |

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
# Optional: For RAG system
QDRANT_URL=your-qdrant-url
OPENAI_API_KEY=your-api-key
DATABASE_URL=your-postgres-url
```

## 🧪 Testing

### Frontend Tests

```bash
cd frontend
npm run build
```

### Backend Tests

```bash
cd backend
pip install -r requirements.txt
pytest
```

## 📖 Architecture

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

## 🎯 Success Criteria

- ✅ Students can navigate content within 3 clicks
- ✅ AI tutor provides accurate, cited answers
- ✅ All simulation exercises are runnable
- ✅ Mobile and low-bandwidth accessible
- ✅ 99% uptime during peak hours

## 📝 License

Open content - see LICENSE file

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a PR

## 📞 Support

- Documentation: [Link to docs]
- GitHub Issues: [Report bugs]
- Community: [Discussion forum]

---

**Built with ❤️ for Panaversity**
