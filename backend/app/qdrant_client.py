from dotenv import load_dotenv
import os
from qdrant_client import QdrantClient, models

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "humanoid_robotics_chunks"

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

async def create_collection_if_not_exists():
    collections = client.get_collections().collections
    if not any(c.name == COLLECTION_NAME for c in collections):
        client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
        print(f"Collection '{COLLECTION_NAME}' created.")
    else:
        print(f"Collection '{COLLECTION_NAME}' already exists.")

async def upsert_vectors(vectors: list[list[float]], payloads: list[dict]):
    points = []
    for i, vector in enumerate(vectors):
        points.append(models.PointStruct(
            id=payloads[i]["chunk_id"], # Using chunk_id as point id
            vector=vector,
            payload=payloads[i]
        ))
    
    operation_info = client.upsert(
        collection_name=COLLECTION_NAME,
        wait=True,
        points=points
    )
    return operation_info

async def search_qdrant(query_embedding: list[float], limit: int = 5) -> list[models.ScoredPoint]:
    search_result = client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=limit,
        with_payload=True
    )
    return search_result
