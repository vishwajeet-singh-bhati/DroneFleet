PY := .venv/bin/python
.PHONY: setup run backend test demo scenarios frontend-build clean

setup:            ## create venv + install backend and frontend deps
	python3 -m venv .venv && .venv/bin/pip install -q -r backend/requirements.txt
	cd frontend && npm install --no-audit --no-fund

run:              ## backend :8000 + frontend :5173
	./start.sh

backend:          ## backend only
	./start.sh backend

test:             ## full backend test-suite
	cd backend && ../$(PY) -m pytest -q

demo:             ## narrated headless demo: make demo SCENARIO=human_takeover
	$(PY) scripts/run_demo.py --scenario $(or $(SCENARIO),human_takeover)

scenarios:        ## run all 8 scenarios headless
	$(PY) scripts/run_demo.py --all

frontend-build:   ## typecheck + production build
	cd frontend && npm run build

clean:
	rm -rf frontend/dist backend/.pytest_cache data/*.db
