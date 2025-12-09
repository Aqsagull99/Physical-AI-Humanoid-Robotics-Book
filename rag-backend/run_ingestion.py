#!/usr/bin/env python3
"""
Script to run content ingestion directly without starting the full server
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.config import settings
from services.retrieval import get_retrieval_service
from core.logging import logger

def run_ingestion():
    """Run the content ingestion process directly"""
    print("Starting content ingestion process...")
    print(f"Loading content from: {settings.book_content_path}")

    try:
        # Get the retrieval service and initialize the store
        retrieval_service = get_retrieval_service()
        logger.info("Initializing vector store with book content...")

        # Initialize the vector store with book content
        retrieval_service.initialize_store()

        print("✅ Content ingestion completed successfully!")
        print("The book content has been loaded into Qdrant.")
        return True

    except Exception as e:
        logger.error(f"Error during ingestion: {str(e)}")
        print(f"❌ Content ingestion failed: {str(e)}")
        return False

if __name__ == "__main__":
    success = run_ingestion()
    if success:
        print("\n🎉 Content ingestion process completed!")
        print("Your book content should now be stored in Qdrant and available for retrieval.")
    else:
        print("\n❌ Content ingestion process failed.")
        sys.exit(1)