import openai
from typing import Optional, AsyncGenerator
from agents import Runner

from src.agents.book_rag_agent.agent import book_rag_agent
from src.agents.book_rag_agent.settings import settings

async def ask_agent(query: str, selected_text: Optional[str] = None) -> AsyncGenerator[str, None]:
    """
    Invokes the BookRAGAgent with the user's query.
    Handles client injection for Gemini compatibility via global settings.
    """
    
    # Construct input
    if selected_text:
        input_text = f"Context: {selected_text}\n\nQuestion: {query}"
    else:
        input_text = query
        
    # Run the agent
    result = await Runner.run(
        book_rag_agent, 
        input=input_text
    )
    
    if hasattr(result, "final_output"):
        yield str(result.final_output)
    else:
        yield str(result)
