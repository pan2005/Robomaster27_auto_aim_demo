from __future__ import annotations

import json
import sqlite3
from datetime import datetime
from pathlib import Path

from .metrics import velocity_metrics


SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    path TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    kind TEXT NOT NULL,
    modified TEXT NOT NULL,
    samples INTEGER,
    state TEXT,
    reason INTEGER,
    score REAL,
    valid INTEGER,
    metadata_json TEXT NOT NULL
)
"""


class ExperimentIndex:
    def __init__(self, experiments_root: Path):
        self.root = experiments_root.resolve()
        self.database = self.root / "experiment_index.sqlite3"
        self.root.mkdir(parents=True, exist_ok=True)
        with sqlite3.connect(self.database) as connection:
            connection.execute(SCHEMA)

    def sync(self) -> None:
        with sqlite3.connect(self.database) as connection:
            for (stored_path,) in connection.execute("SELECT path FROM runs").fetchall():
                if not Path(stored_path).exists():
                    connection.execute("DELETE FROM runs WHERE path = ?", (stored_path,))
            for metadata_path in self.root.rglob("*.json"):
                if metadata_path.name.endswith("_summary.json"):
                    continue
                try:
                    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    continue
                csv_path = metadata_path.with_suffix(".csv")
                if metadata.get("experiment") == "gravity":
                    kind = "pitch-gravity"
                elif metadata.get("experiment") == "pitch-control":
                    kind = "pitch-control"
                else:
                    kind = "velocity" if "mode" in metadata else "sysid"
                metrics = None
                if kind == "velocity" and csv_path.exists():
                    try:
                        metrics = velocity_metrics(csv_path)
                    except Exception:
                        metrics = None
                status = metadata.get("final_status") or []
                state = str(status[0]) if status else "unknown"
                reason = int(status[-1]) if status else 0
                connection.execute(
                    """INSERT OR REPLACE INTO runs
                       (path,name,kind,modified,samples,state,reason,score,valid,metadata_json)
                       VALUES (?,?,?,?,?,?,?,?,?,?)""",
                    (str(csv_path), csv_path.stem, kind,
                     datetime.fromtimestamp(metadata_path.stat().st_mtime).isoformat(timespec="seconds"),
                     int(metadata.get("sample_count", 0)), state, reason,
                     None if metrics is None else float(metrics["score"]),
                     None if metrics is None else int(bool(metrics["valid"])),
                     json.dumps(metadata, ensure_ascii=False)),
                )
            connection.commit()

    def rows(self) -> list[tuple]:
        with sqlite3.connect(self.database) as connection:
            return connection.execute(
                "SELECT kind,name,modified,samples,state,reason,score,valid,path "
                "FROM runs ORDER BY modified DESC"
            ).fetchall()
