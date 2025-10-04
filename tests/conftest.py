import pytest
from asyncio_for_robotics.core._logger import setup_logger

@pytest.fixture(scope="session", autouse=True)
def configure_logging():
    """Configure logging once for all tests."""
    setup_logger(debug_path="tests")
 
