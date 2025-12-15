import asyncio
import argparse
import sys
import logging

from src.agents.book_rag_agent.interface import ask_agent
from src.agents.book_rag_agent.utils import setup_logging

logger = setup_logging()

async def main():
    parser = argparse.ArgumentParser(description="Book RAG Agent CLI")
    parser.add_argument("--query", type=str, help="The user's question")
    parser.add_argument("--selected-text", type=str, help="Context override")
    parser.add_argument("--interactive", action="store_true", help="Interactive mode")
    
    args = parser.parse_args()
    
    if args.interactive:
        print("--- Book RAG Agent (Interactive) ---")
        while True:
            try:
                q = input("\nQuery: ")
                if q.lower() in ["exit", "quit"]:
                    break
                
                print("\nThinking...", end="", flush=True)
                print("\rResponse: ", end="", flush=True)
                
                async for chunk in ask_agent(q, selected_text=args.selected_text):
                    print(chunk, end="", flush=True)
                print()
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f"Error: {e}")
    
    elif args.query:
        try:
            async for chunk in ask_agent(args.query, selected_text=args.selected_text):
                print(chunk, end="", flush=True)
            print()
        except Exception as e:
            logger.error(f"Error: {e}")
            sys.exit(1)
    else:
        parser.print_help()

if __name__ == "__main__":
    asyncio.run(main())
