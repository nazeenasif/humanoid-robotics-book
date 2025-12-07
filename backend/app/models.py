from pydantic import BaseModel
from typing import List, Optional

class QueryModel(BaseModel):
    question: str
    selected_text: Optional[str] = None

class Source(BaseModel):
    title: str
    section: str
    url: str
    score: float

class RagResponse(BaseModel):
    answer: str
    sources: List[Source]

class DocumentMetadata(BaseModel):
    title: str
    section: str
    url: str
