from agents import Agent, OpenAIChatCompletionsModel

from backend.agents.book_rag_agent.tools import retrieve_book_context
from backend.agents.book_rag_agent.guardrails import relevance_guardrail
from backend.agents.book_rag_agent.settings import settings

# Instructions for the agent
INSTRUCTIONS = """You are an expert technical assistant for the 'Robotics Book'.
Your goal is to answer user questions using ONLY information from the book.
You have access to a tool 'retrieve_book_context' that searches the book's content.

1. ALWAYS use the 'retrieve_book_context' tool first if the user asks a question about robotics or the book's content.
2. If the answer is not in the retrieved chunks, say "I cannot find the answer in the book."
3. Do not use outside knowledge.
4. If the user provides a "Context" block in their message, prioritize that information.
"""

# Configure Model with Gemini Client
gemini_model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash-lite",
    openai_client=settings.get_openai_client()
)

# Define the Agent
book_rag_agent = Agent(
    name="BookRAGAgent",
    instructions=INSTRUCTIONS,
    model=gemini_model,
    tools=[retrieve_book_context],
    input_guardrails=[relevance_guardrail]
)
