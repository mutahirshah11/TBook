import openai
from typing import Optional, AsyncGenerator, Dict, Any
from agents import Runner
from backend.agents.book_rag_agent.agent import book_rag_agent
from backend.agents.book_rag_agent.settings import settings

async def ask_agent(query: str, selected_text: Optional[str] = None, user_profile: Optional[Dict[str, Any]] = None) -> AsyncGenerator[str, None]:
    """
    Invokes the BookRAGAgent with the user's query.
    Handles client injection for Gemini compatibility via global settings.
    """
    
    # Construct input
    input_parts = []
    
    # Add profile context if available
    if user_profile:
        proficiency = user_profile.get("python_proficiency")
        role = user_profile.get("developer_role")
        if proficiency or role:
            input_parts.append(f"User Context: Python Level: {proficiency}, Role: {role}")

    if selected_text:
        input_parts.append(f"Context: {selected_text}")
    
    input_parts.append(f"Question: {query}")
    
    input_text = "\n\n".join(input_parts)
        
    # Run the agent
    result = await Runner.run(
        book_rag_agent, 
        input=input_text
    )
    
    if hasattr(result, "final_output"):
        yield str(result.final_output)
    else:
        yield str(result)
