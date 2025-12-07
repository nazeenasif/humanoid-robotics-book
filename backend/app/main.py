from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

from .rag_pipeline import rag_pipeline
from .models import QueryModel, RagResponse

load_dotenv()

app = FastAPI()

# Configure CORS for Vercel deployment
origins = [
    "*", # Allow all origins for now, will restrict to Vercel domain later
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/healthcheck")
async def healthcheck():
    return {"status": "ok"}

@app.post("/ask", response_model=RagResponse)
async def ask_question(query: QueryModel):
    try:
        response = await rag_pipeline(query.question, query.selected_text)
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/embed")
async def embed_text(text: dict):
    # This endpoint is internal and will be used by the ingestion script
    # Implementation will be in rag_pipeline.py
    return {"message": "Embedding not yet implemented via API"}
