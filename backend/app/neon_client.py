from dotenv import load_dotenv
import os
import asyncpg

load_dotenv()

NEON_URL = os.getenv("NEON_URL")

async def get_connection():
    return await asyncpg.connect(NEON_URL)

async def create_tables():
    conn = None
    try:
        conn = await get_connection()
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS documents (
                id SERIAL PRIMARY KEY,
                title TEXT,
                section TEXT,
                url TEXT
            );
            CREATE TABLE IF NOT EXISTS chunks (
                id SERIAL PRIMARY KEY,
                doc_id INTEGER REFERENCES documents(id),
                chunk TEXT
            );
            CREATE TABLE IF NOT EXISTS chat_logs (
                id SERIAL PRIMARY KEY,
                question TEXT,
                answer TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """)
        print("Tables created or already exist.")
    except Exception as e:
        print(f"Error creating tables: {e}")
    finally:
        if conn:
            await conn.close()

async def insert_document(title: str, section: str, url: str) -> int:
    conn = await get_connection()
    doc_id = await conn.fetchval(
        "INSERT INTO documents(title, section, url) VALUES($1, $2, $3) RETURNING id",
        title, section, url
    )
    await conn.close()
    return doc_id

async def insert_chunk(doc_id: int, chunk_text: str) -> int:
    conn = await get_connection()
    chunk_id = await conn.fetchval(
        "INSERT INTO chunks(doc_id, chunk) VALUES($1, $2) RETURNING id",
        doc_id, chunk_text
    )
    await conn.close()
    return chunk_id

async def fetch_document_metadata(doc_id: int):
    conn = await get_connection()
    record = await conn.fetchrow(
        "SELECT title, section, url FROM documents WHERE id = $1",
        doc_id
    )
    await conn.close()
    return record

async def fetch_chunk_text(chunk_id: int) -> str | None:
    conn = await get_connection()
    record = await conn.fetchrow(
        "SELECT chunk FROM chunks WHERE id = $1",
        chunk_id
    )
    await conn.close()
    return record['chunk'] if record else None

async def insert_chat_log(question: str, answer: str):
    conn = await get_connection()
    await conn.execute(
        "INSERT INTO chat_logs(question, answer) VALUES($1, $2)",
        question, answer
    )
    await conn.close()
