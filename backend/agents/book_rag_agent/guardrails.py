from agents import Agent, Runner, input_guardrail, GuardrailFunctionOutput, OpenAIChatCompletionsModel
from backend.agents.book_rag_agent.models import RelevanceOutput
from backend.agents.book_rag_agent.settings import settings

# Configure Model with Gemini Client
gemini_model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=settings.get_openai_client()
)

# Define the Guardrail Agent
guardrail_agent = Agent(
    name="RelevanceGuardrail",
    instructions="""You are a relevance checker for a technical book about robotics.
Check if the user's query is related to robotics, AI, or the book's domain.
If the query is general small talk (like "hello"), it is relevant (pass it through).
If the query is completely unrelated (e.g. "how to bake a cake"), it is NOT relevant.
""",
    output_type=RelevanceOutput,
    model=gemini_model
)

@input_guardrail
async def relevance_guardrail(ctx, agent, input_data) -> GuardrailFunctionOutput:
    """
    Checks if the input is relevant using a separate lightweight agent.
    """
    
    result = await Runner.run(
        guardrail_agent, 
        input=input_data
    )
    
    relevance: RelevanceOutput = result.final_output
    
    if not relevance.is_relevant:
        return GuardrailFunctionOutput(
            tripwire_triggered=True,
            output_info="I can only answer questions about the Robotics Book."
        )
        
    return GuardrailFunctionOutput(
        tripwire_triggered=False,
        output_info=None
    )
