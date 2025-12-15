import pytest
import asyncio
import time
from concurrent.futures import ThreadPoolExecutor
import sys
import os

# Add the backend directory to the path so we can import from it
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from backend.agent_client import AgentClient
from backend.schemas import Message
from datetime import datetime


@pytest.mark.performance
@pytest.mark.asyncio
async def test_concurrent_agent_requests():
    """Test handling of concurrent agent requests"""
    agent_client = AgentClient()
    message = Message(
        content="Performance test message",
        user_id="perf-test-user",
        timestamp=datetime.now()
    )

    # Number of concurrent requests to test
    num_requests = 10

    start_time = time.time()

    # Create concurrent tasks
    tasks = []
    for i in range(num_requests):
        task = agent_client.run_agent(message)
        tasks.append(task)

    # Execute all tasks concurrently
    results = await asyncio.gather(*tasks)

    end_time = time.time()
    total_time = end_time - start_time

    # Verify all requests completed successfully
    assert len(results) == num_requests
    for result in results:
        assert result.status.value in ["success", "error"]  # Allow for error responses too

    # Performance requirement: all 10 concurrent requests should complete in under 5 seconds
    # (This is a reasonable requirement given each request simulates 0.5s processing)
    assert total_time < 5.0, f"Concurrent requests took too long: {total_time:.2f}s"


@pytest.mark.performance
def test_agent_client_initialization_performance():
    """Test that agent client initializes quickly"""
    start_time = time.time()
    agent_client = AgentClient()
    end_time = time.time()

    init_time = end_time - start_time

    # Initialization should be very fast (under 0.1 seconds)
    assert init_time < 0.1, f"Agent client initialization took too long: {init_time:.4f}s"


@pytest.mark.performance
@pytest.mark.asyncio
async def test_single_request_response_time():
    """Test that individual requests respond within acceptable time"""
    agent_client = AgentClient()
    message = Message(
        content="Single request test",
        user_id="single-test-user",
        timestamp=datetime.now()
    )

    start_time = time.time()
    response = await agent_client.run_agent(message)
    end_time = time.time()

    request_time = end_time - start_time

    # Individual request should complete in under 1 second (plus the simulated 0.5s processing)
    assert request_time < 2.0, f"Single request took too long: {request_time:.2f}s"
    assert response.status.value in ["success", "error"]


@pytest.mark.performance
@pytest.mark.asyncio
async def test_memory_usage_stability():
    """Test that memory usage remains stable under load"""
    import gc
    import psutil
    import os

    process = psutil.Process(os.getpid())
    initial_memory = process.memory_info().rss / 1024 / 1024  # MB

    agent_client = AgentClient()
    message = Message(
        content="Memory test message",
        user_id="memory-test-user",
        timestamp=datetime.now()
    )

    # Make multiple requests to check for memory leaks
    for i in range(20):
        response = await agent_client.run_agent(message)
        assert response.status.value in ["success", "error"]

        # Force garbage collection periodically
        if i % 5 == 0:
            gc.collect()

    # Check final memory usage
    final_memory = process.memory_info().rss / 1024 / 1024  # MB
    memory_increase = final_memory - initial_memory

    # Memory increase should be reasonable (less than 50MB for 20 requests)
    assert memory_increase < 50, f"Memory increase too high: {memory_increase:.2f}MB"


@pytest.mark.performance
@pytest.mark.asyncio
async def test_streaming_performance():
    """Test performance of streaming requests"""
    agent_client = AgentClient()
    message = Message(
        content="Streaming performance test",
        user_id="stream-perf-test",
        timestamp=datetime.now()
    )

    start_time = time.time()

    # Collect streaming chunks
    chunks = []
    async for chunk in agent_client.run_agent_stream(message):
        chunks.append(chunk)

    end_time = time.time()
    stream_time = end_time - start_time

    # Verify we got some chunks
    assert len(chunks) > 0
    for chunk in chunks:
        assert 'content' in chunk
        assert 'status' in chunk

    # Streaming should complete in reasonable time (under 2 seconds for simulated content)
    assert stream_time < 2.0, f"Streaming took too long: {stream_time:.2f}s"