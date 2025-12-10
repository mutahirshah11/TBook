# Quickstart: Book RAG Agent

## Prerequisites

1. **Python 3.10+**
2. **UV** (Package Manager)
3. **API Keys**:
   - Gemini (`GEMINI_API_KEY`)
   - Qdrant (`QDRANT_URL`, `QDRANT_API_KEY`)
   - Cohere (`COHERE_API_KEY`)

## Setup

1. **Install Dependencies**:
   ```bash
   uv sync
   ```

2. **Configure Environment**:
   Create `.env` in the root:
   ```env
   GEMINI_API_KEY=your_gemini_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   COHERE_API_KEY=your_cohere_key
   ```

## Usage

### Interactive Mode (Recommended)
```bash
uv run scripts/rag_agent.py --interactive
```

### Single Query
```bash
uv run scripts/rag_agent.py --query "What is the definition of physical AI?"
```

### With Selected Text Override
```bash
uv run scripts/rag_agent.py --query "Explain this" --selected-text "Robots use sensors to perceive the world."
```

## Running Tests
```bash
uv run pytest scripts/test_rag_agent.py
```
