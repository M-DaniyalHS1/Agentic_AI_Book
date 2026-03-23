"""
SQLAlchemy database client singleton
"""
import os
from dotenv import load_dotenv
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session
from contextlib import contextmanager

from src.database import Base

load_dotenv()

# Create engine
engine = create_engine(
    os.getenv("DATABASE_URL"),
    pool_pre_ping=True,
    echo=False
)

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


@contextmanager
def get_db_session() -> Session:
    """
    Get database session context manager
    
    Usage:
        with get_db_session() as db:
            user = db.query(User).filter(...).first()
    """
    db = SessionLocal()
    try:
        yield db
        db.commit()
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


def init_db():
    """Initialize database tables (for development only)"""
    Base.metadata.create_all(bind=engine)


def close_db():
    """Dispose of engine connections"""
    engine.dispose()
