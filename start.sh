#!/usr/bin/env bash
# One command to run everything: backend (FastAPI :8000) + frontend (Vite :5173).
#   ./start.sh            install deps if needed, start both
#   ./start.sh backend    backend only
#   PORT=8000 UI_PORT=5173 ./start.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${PORT:-8000}"; UI_PORT="${UI_PORT:-5173}"
VENV="$ROOT/.venv"; PY="$VENV/bin/python"

if [ ! -x "$PY" ]; then
  echo "▶ creating virtualenv"; python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q --upgrade pip
fi
if ! "$PY" -c "import fastapi, uvicorn, sqlalchemy, sklearn, numpy" 2>/dev/null; then
  echo "▶ installing backend dependencies"; "$VENV/bin/pip" install -q -r "$ROOT/backend/requirements.txt"
fi

pids=()
cleanup() { for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "▶ backend  http://localhost:$PORT   (API docs: /docs)"
( cd "$ROOT/backend" && exec "$PY" -m uvicorn app.main:create_app --factory --host 0.0.0.0 --port "$PORT" ) &
pids+=($!)

if [ "${1:-}" = "backend" ]; then wait; exit 0; fi

if command -v npm >/dev/null 2>&1; then
  [ -d "$ROOT/frontend/node_modules" ] || { echo "▶ installing frontend dependencies"; (cd "$ROOT/frontend" && npm install --no-audit --no-fund --silent); }
  echo "▶ frontend http://localhost:$UI_PORT"
  ( cd "$ROOT/frontend" && BACKEND_URL="http://127.0.0.1:$PORT" exec npx vite --host 0.0.0.0 --port "$UI_PORT" ) &
  pids+=($!)
else
  echo "⚠ npm not found — running backend only (API docs at http://localhost:$PORT/docs)"
fi
wait
