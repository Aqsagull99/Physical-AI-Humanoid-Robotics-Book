#!/usr/bin/env python3
"""
Script to check what content is actually in the vector store
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from services.vector_store import get_vector_store
from core.logging import logger
from services.content_loader import get_content_loader

def check_content():
    """Check what content is available"""
    print("Checking content in the vector store...")

    try:
        # Check what content was loaded from the source
        content_loader = get_content_loader()
        content = content_loader.load_content()
        print(f"Loaded {len(content)} content chunks from source")

        # Check a few sample contents
        print("\nSample content chunks:")
        for i, item in enumerate(content[:3]):
            print(f"  {i+1}. Content preview: {item.get('content', '')[:200]}...")
            print(f"     File: {item.get('source_file', 'unknown')}")
            print(f"     Metadata keys: {list(item.get('metadata', {}).keys())}")
            print()

        # Check vector store
        vector_store = get_vector_store()
        if vector_store.qdrant_client:
            # Get a few sample points to see what's in Qdrant
            try:
                count_result = vector_store.qdrant_client.count(
                    collection_name=vector_store.collection_name
                )
                print(f"Total points in Qdrant: {count_result.count}")

                # Try to get a few sample points
                if count_result.count > 0:
                    scroll_result = vector_store.qdrant_client.scroll(
                        collection_name=vector_store.collection_name,
                        limit=5,
                        with_payload=True,
                        with_vectors=False
                    )
                    print(f"\nSample points from Qdrant ({len(scroll_result[0])} retrieved):")
                    for i, (point_id, payload) in enumerate(zip(scroll_result[0], scroll_result[1])):
                        print(f"  {i+1}. ID: {point_id}")
                        print(f"     Content preview: {payload.get('content', '')[:200]}...")
                        print(f"     File path: {payload.get('file_path', 'N/A')}")
                        print(f"     Metadata keys: {list(payload.keys())}")
                        print()

            except Exception as e:
                logger.error(f"Error checking Qdrant content: {e}")
                import traceback
                traceback.print_exc()
                return False

    except Exception as e:
        logger.error(f"Error checking content: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

    return True

if __name__ == "__main__":
    success = check_content()
    if success:
        print("\n✅ Content check completed!")
    else:
        print("\n❌ Content check failed.")
        sys.exit(1)