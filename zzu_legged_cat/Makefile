PYTHON ?= $(if $(wildcard .venv-mjbatch/bin/python),.venv-mjbatch/bin/python,python3)
SECONDS ?= 10

.DEFAULT_GOAL := help

.PHONY: help build gui headless check check-docs check-portable test-core test-rl backend-smoke catalog-logs

help:
	@printf '%s\n' 'build          Build the native controller' \
	  'gui            Run the native GUI' \
	  'headless       Run the native controller (SECONDS=10)' \
	  'check          Check repository layout and C portability' \
	  'test-core      Run Python core and backend regression tests' \
	  'test-rl        Run all included spin/pitch regression tests' \
	  'backend-smoke  Step the MuJoCo reference backend' \
	  'catalog-logs   Index local raw logs without moving them'

build:
	mkdir -p logs
	$(MAKE) -C control

gui: build
	./controller

headless: build
	./controller --headless $(SECONDS)

check: check-docs check-portable

check-docs:
	$(PYTHON) tools/check_repository.py

check-portable:
	$(MAKE) -C control check-portable

test-core:
	$(PYTHON) -m pytest -q scripts/tools/test_physics_framework.py \
	  scripts/tools/test_spin_joint_pd.py scripts/tools/test_spin_wrench.py

test-rl:
	$(PYTHON) -m pytest -q scripts/tools/test_physics_framework.py \
	  scripts/tools/test_spin_*.py

backend-smoke:
	$(PYTHON) scripts/tools/check_physics_backend.py --envs 4 --ticks 100

catalog-logs:
	mkdir -p logs
	$(PYTHON) tools/catalog_logs.py
