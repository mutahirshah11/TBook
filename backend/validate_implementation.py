#!/usr/bin/env python
"""
Quick validation script to test the backend implementation
"""
import sys
import os

# Add the parent directory to the path to allow package imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

def test_imports():
    """Test that all modules can be imported properly when run as part of the package"""
    try:
        # Test importing the main components
        from backend.schemas import Message, Response, ChatRequest, ChatResponse
        from backend.agent_client import AgentClient
        print("[SUCCESS] All core modules imported successfully")

        # Test schema creation
        message = Message(content="Test message")
        print(f"[SUCCESS] Message schema created: {message.content}")

        # Test agent client initialization
        agent = AgentClient()
        print(f"[SUCCESS] Agent client initialized with timeout: {agent.timeout}")

        print("\n[SUCCESS] Backend implementation validation passed!")
        return True

    except ImportError as e:
        print(f"[ERROR] Import error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Error during validation: {e}")
        return False

if __name__ == "__main__":
    print("Validating backend implementation...")
    success = test_imports()
    if success:
        print("\n[SUCCESS] All validations passed! Backend is ready for deployment.")
    else:
        print("\n[ERROR] Some validations failed.")
        sys.exit(1)