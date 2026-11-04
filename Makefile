UV ?= uv
REQ = $(UV) run --python 3.11 --no-project python tools/requirements.py
PORT ?= 5111

.PHONY: requirements-init requirements-check requirements-sync docs serve
requirements-init:
	$(REQ) init
requirements-check:
	$(REQ) check
requirements-sync:
	$(REQ) sync
docs:
	$(REQ) docs
serve:
	$(REQ) serve --port $(PORT)
