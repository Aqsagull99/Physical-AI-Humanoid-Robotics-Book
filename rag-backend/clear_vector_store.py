#!/usr/bin/env python3
"""
Script to clear the Qdrant vector store and re-ingest content
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from services.vector_store import get_vector_store
from core.logging import logger

def clear_and_rebuild():
    """Clear the vector store and rebuild it"""
    print("Clearing vector store...")

    try:
        vector_store = get_vector_store()
        vector_store.clear()
        print("✅ Vector store cleared successfully!")

    except Exception as e:
        logger.error(f"Error clearing vector store: {str(e)}")
        return False

    return True

if __name__ == "__main__":
    success = clear_and_rebuild()
    if success:
        print("\n✅ Vector store cleared! Now run 'python run_ingestion.py' to re-ingest content.")
    else:
        print("\n❌ Failed to clear vector store.")
        sys.exit(1)