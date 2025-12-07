
from dotenv import load_dotenv
import os
import google.generativeai as genai
from sentence_transformers import SentenceTransformer

from .qdrant_client import search_qdrant
from .neon_client import fetch_document_metadata, fetch_chunk_text
from .models import RagResponse, Source

load_dotenv()

genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

# Initialize the sentence-transformer model globally to load once
embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

async def embed_text(text: str) -> list[float]:
    # The SentenceTransformer model is not async, run it in a thread pool executor if needed in a real async app
    return embedding_model.encode(text).tolist()

async def rag_pipeline(question: str, selected_text: str | None = None) -> RagResponse:
    query_embedding = await embed_text(question)

    # Search Qdrant for relevant chunks
    qdrant_results = await search_qdrant(query_embedding)

    context_chunks = []
    sources = []

    for result in qdrant_results:
        doc_id = result.payload["doc_id"]
        chunk_id = result.payload["chunk_id"]

        document_metadata = await fetch_document_metadata(doc_id)
        chunk_text = await fetch_chunk_text(chunk_id)

        if document_metadata and chunk_text:
            context_chunks.append(chunk_text)
            source = Source(
                title=document_metadata.title,
                section=document_metadata.section,
                url=document_metadata.url,
                score=result.score
            )
            sources.append(source)

    # Optionally include selected text from the frontend
    if selected_text:
        context_chunks.insert(0, selected_text)

    context = "\n\n".join(context_chunks)

    # Format messages for Gemini
    prompt_parts = [
        "You are a helpful assistant for the Humanoid Robotics Book. Answer questions based on the provided context only.\n\n",
        f"Context: {context}\n\nQuestion: {question}"
    ]

    try:
        model = genai.GenerativeModel('gemini-pro') # Using gemini-pro for chat
        response = model.generate_content(prompt_parts)
        answer = response.text
    except Exception as e:
        raise Exception(f"Error calling Gemini API: {e}")

    return RagResponse(answer=answer, sources=sources)
