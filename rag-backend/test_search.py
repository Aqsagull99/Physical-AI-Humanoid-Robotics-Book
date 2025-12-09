#!/usr/bin/env python3
"""
Script to test direct search functionality
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from services.retrieval import get_retrieval_service
from services.vector_store import get_vector_store
from core.logging import logger

def test_search():
    """Test search functionality directly"""
    print("Testing search functionality...")

    try:
        # Test with the retrieval service
        retrieval_service = get_retrieval_service()

        # Test different queries
        test_queries = [
            "ROS",
            "ROS 2",
            "Robot Operating System",
            "communication infrastructure",
            "robotics",
            "What is ROS?"
        ]

        for query in test_queries:
            print(f"\n--- Testing query: '{query}' ---")
            results = retrieval_service.retrieve_relevant_content(query=query, top_k=3)
            print(f"Results found: {len(results)}")

            for i, result in enumerate(results):
                print(f"  {i+1}. Similarity: {result.get('similarity', 0):.4f}")
                print(f"     Content preview: {result.get('content', '')[:150]}...")
                print(f"     File: {result.get('metadata', {}).get('file_path', 'N/A')}")
                print()

        # Also test direct vector store search
        print("\n--- Testing direct vector store search ---")
        vector_store = get_vector_store()

        # Test with a simple term that should exist
        try:
            direct_results = vector_store.find_similar_texts("ROS", top_k=3)
            print(f"Direct search results for 'ROS': {len(direct_results)}")
            for i, (content_item, similarity) in enumerate(direct_results):
                print(f"  {i+1}. Similarity: {similarity:.4f}")
                print(f"     Content preview: {content_item.get('content', '')[:150]}...")
                print(f"     File: {content_item.get('metadata', {}).get('file_path', 'N/A')}")
                print()
        except Exception as e:
            logger.error(f"Direct search error: {e}")
            import traceback
            traceback.print_exc()

    except Exception as e:
        logger.error(f"Error in search test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

    return True

if __name__ == "__main__":
    success = test_search()
    if success:
        print("\n✅ Search test completed!")
    else:
        print("\n❌ Search test failed.")
        sys.exit(1)