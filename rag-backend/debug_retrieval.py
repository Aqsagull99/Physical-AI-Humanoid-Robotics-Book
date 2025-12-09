#!/usr/bin/env python3
"""
Debug script to test the retrieval process and identify why queries aren't matching
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.config import settings
from services.retrieval import get_retrieval_service
from services.vector_store import get_vector_store
from core.logging import logger
import numpy as np

def debug_retrieval():
    """Debug the retrieval process"""
    print("Debugging retrieval process...")
    print(f"Book content path: {settings.book_content_path}")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Vector store type: {settings.vector_store_type}")
    print(f"Min similarity threshold: {settings.min_similarity_threshold}")

    try:
        # Get the vector store and check if collection exists
        vector_store = get_vector_store()
        print(f"Vector store collection: {vector_store.collection_name}")

        # Test a simple query
        test_query = "What is ROS?"
        print(f"\nTesting query: '{test_query}'")

        # Get retrieval service and test retrieval
        retrieval_service = get_retrieval_service()
        results = retrieval_service.retrieve_relevant_content(query=test_query, top_k=5)

        print(f"Number of results found: {len(results)}")

        if results:
            print("\nTop results:")
            for i, result in enumerate(results):
                print(f"  Result {i+1}:")
                print(f"    ID: {result.get('id', 'N/A')}")
                print(f"    Similarity: {result.get('similarity', 'N/A')}")
                print(f"    Content preview: {result.get('content', '')[:100]}...")
                print(f"    Metadata keys: {list(result.get('metadata', {}).keys())}")
        else:
            print("  No results found - this indicates the issue!")

        # Let's also test with a more specific query that should match
        specific_query = "ROS 2"
        print(f"\nTesting specific query: '{specific_query}'")
        specific_results = retrieval_service.retrieve_relevant_content(query=specific_query, top_k=5)
        print(f"Number of results for specific query: {len(specific_results)}")

        if specific_results:
            print("Specific query results:")
            for i, result in enumerate(specific_results):
                print(f"  Result {i+1}: Similarity={result.get('similarity', 'N/A')}, Content='{result.get('content', '')[:100]}...'")

        # Check if there are any entries in the vector store
        print("\nChecking if vector store has any content...")
        # We'll try to get all points to see what's in there
        if vector_store.qdrant_client:
            try:
                # Count the number of points in the collection
                count_result = vector_store.qdrant_client.count(
                    collection_name=vector_store.collection_name
                )
                print(f"Total points in collection: {count_result.count}")

                # Try to get a few sample points
                if count_result > 0:
                    scroll_result = vector_store.qdrant_client.scroll(
                        collection_name=vector_store.collection_name,
                        limit=3,
                        with_payload=True,
                        with_vectors=False
                    )
                    print(f"Sample points retrieved: {len(scroll_result[0]) if scroll_result else 0}")
                    for i, (point_id, payload) in enumerate(zip(scroll_result[0], scroll_result[1])):
                        print(f"  Sample {i+1}: ID={point_id}, Content preview='{payload.get('content', '')[:100]}...'")
            except Exception as e:
                print(f"Error checking Qdrant collection: {e}")

    except Exception as e:
        logger.error(f"Error during debugging: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

    return True

if __name__ == "__main__":
    success = debug_retrieval()
    if success:
        print("\n✅ Debug completed!")
    else:
        print("\n❌ Debug failed.")
        sys.exit(1)