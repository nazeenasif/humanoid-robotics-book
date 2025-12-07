import asyncio
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models

from app.neon_client import create_tables, insert_document, insert_chunk
from app.qdrant_client import create_collection_if_not_exists, upsert_vectors
from app.rag_pipeline import embed_text # Re-using the embedding function

load_dotenv()

# Initialize clients
qdrant_client_instance = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))

# No specific tokenizer needed for embeddings in this flow

async def ingest_documents(docs_path: str):
    await create_tables()
    await create_collection_if_not_exists()

    markdown_files = []
    for root, _, files in os.walk(docs_path):
        for file in files:
            if file.endswith(".md"):
                markdown_files.append(os.path.join(root, file))

    for file_path in markdown_files:
        print(f"Processing {file_path}...")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        # Extract title and section from file path
        relative_path = os.path.relpath(file_path, docs_path)
        parts = relative_path.split(os.sep)

        title = os.path.basename(file_path).replace(".md", "").replace("-", " ").title()
        section = parts[0] if len(parts) > 1 else ""
        url = f"/{relative_path.replace('.md', '')}"

        doc_id = await insert_document(title, section, url)

        # Split into chunks (simple by paragraphs for now)
        chunks = content.split("\n\n")
        processed_chunks = []
        chunk_payloads = []

        for i, chunk_text in enumerate(chunks):
            if chunk_text.strip():
                # Using basic character limit for chunking
                if len(chunk_text) > 1500:
                    sub_chunks = [chunk_text[i:i+1500] for i in range(0, len(chunk_text), 1500)]
                    for sub_chunk_text in sub_chunks:
                        processed_chunks.append(sub_chunk_text)
                        chunk_payloads.append({"doc_id": doc_id, "chunk_text": sub_chunk_text})
                else:
                    processed_chunks.append(chunk_text)
                    chunk_payloads.append({"doc_id": doc_id, "chunk_text": chunk_text})

        # Embed chunks and store in Qdrant and Neon
        if processed_chunks:
            chunk_ids = []
            for chunk_text in processed_chunks:
                chunk_id = await insert_chunk(doc_id, chunk_text)
                chunk_ids.append(chunk_id)

            vectors = []
            for i, chunk_text in enumerate(processed_chunks):
                embedding = await embed_text(chunk_text)
                vectors.append(embedding)

            # Prepare payloads with actual chunk_id from Neon
            qdrant_payloads = []
            for i, payload in enumerate(chunk_payloads):
                qdrant_payloads.append({
                    "doc_id": payload["doc_id"],
                    "chunk_id": chunk_ids[i],
                    "title": title,
                    "section": section,
                    "url": url
                })

            await upsert_vectors(vectors, qdrant_payloads)
            print(f"Ingested {len(processed_chunks)} chunks for {file_path}")


if __name__ == "__main__":
    docs_folder = "D:/Naz/Hackathon/humanoid-robotics-book/docs"  # Adjust this path as needed
    asyncio.run(ingest_documents(docs_folder))
