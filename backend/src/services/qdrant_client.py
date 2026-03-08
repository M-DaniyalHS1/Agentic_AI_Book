"""
Qdrant Vector Database Client Configuration
Serverless-compatible: Requires Qdrant Cloud (no local Qdrant)
"""
import os
from dotenv import load_dotenv

load_dotenv()

# Qdrant configuration - MUST use Qdrant Cloud for serverless
QDRANT_URL = os.getenv("QDRANT_URL", "")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")

# Embedding configuration - Updated for OpenAI embeddings
EMBEDDING_DIMENSION = 1536  # OpenAI text-embedding-ada-002
COLLECTION_NAME = "textbook_content"


def get_qdrant_client():
    """Get Qdrant client instance (lazy import for serverless)"""
    from qdrant_client import QdrantClient
    from qdrant_client.models import Distance, VectorParams
    
    if not QDRANT_URL:
        raise ValueError("QDRANT_URL environment variable is required. Set up Qdrant Cloud at https://cloud.qdrant.io")
    
    return QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        prefer_grpc=False
    )


def init_qdrant_collection(client=None) -> bool:
    """
    Initialize Qdrant collection for textbook content embeddings
    Returns True if collection was created, False if it already exists
    """
    from qdrant_client.models import Distance, VectorParams
    
    if client is None:
        client = get_qdrant_client()

    try:
        # Check if collection exists
        collections = client.get_collections().collections
        existing = [c.name for c in collections]

        if COLLECTION_NAME in existing:
            print(f"Collection '{COLLECTION_NAME}' already exists")
            return False

        # Create collection
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSION,
                distance=Distance.COSINE
            ),
            optimizers_config={
                "indexing_threshold": 20000
            }
        )

        # Create payload index for faster filtering
        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="module_slug",
            field_schema="keyword"
        )
        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="chapter_slug",
            field_schema="keyword"
        )
        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name="section_slug",
            field_schema="keyword"
        )

        print(f"Collection '{COLLECTION_NAME}' created successfully")
        return True

    except Exception as e:
        print(f"Error initializing Qdrant collection: {e}")
        raise


def get_or_create_client():
    """Get client and ensure collection exists"""
    client = get_qdrant_client()
    init_qdrant_collection(client)
    return client
