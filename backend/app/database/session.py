"""Engine/session factory. SQLite by default; in-memory for tests."""
from __future__ import annotations

from pathlib import Path

from sqlalchemy import create_engine
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session, sessionmaker
from sqlalchemy.pool import StaticPool

from app.database.models import Base


def make_engine(url: str = "sqlite:///./data/dronefleet.db") -> Engine:
    if url.startswith("sqlite") and ":memory:" in url:
        eng = create_engine(url, connect_args={"check_same_thread": False}, poolclass=StaticPool)
    elif url.startswith("sqlite"):
        path = url.replace("sqlite:///", "", 1)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        eng = create_engine(url, connect_args={"check_same_thread": False})
    else:
        eng = create_engine(url)
    Base.metadata.create_all(eng)
    return eng


def make_session_factory(engine: Engine) -> sessionmaker[Session]:
    return sessionmaker(bind=engine, expire_on_commit=False)
