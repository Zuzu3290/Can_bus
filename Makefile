# ═══════════════════════════════════════════════════════════
#  CAN Bus 3D Vehicle Simulator — Makefile
#  Targets:  cpp  |  python  |  all  |  clean  |  deps
# ═══════════════════════════════════════════════════════════

CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
LDFLAGS  := -lpthread
TARGET   := canbus_3d
SRC      := canbus_3d.cpp

PY       := python3
PY_SRC   := canbus_3d.py

.PHONY: all cpp python deps clean help

## Default: build C++ and check Python
all: cpp
	@echo ""
	@echo "  ╔══════════════════════════════════════════════╗"
	@echo "  ║  Build complete. Run with:                   ║"
	@echo "  ║    make cpp     →  ./canbus_3d               ║"
	@echo "  ║    make python  →  python3 canbus_3d.py      ║"
	@echo "  ╚══════════════════════════════════════════════╝"

## Build C++ binary
cpp: $(TARGET)

$(TARGET): $(SRC)
	@echo "  [C++] Compiling $(SRC)..."
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)
	@echo "  [C++] Binary ready: ./$(TARGET)"

## Run Python version
python:
	@echo "  [PY]  Launching Python 3D visualizer..."
	$(PY) $(PY_SRC)

## Run C++ version (build first if needed)
run: $(TARGET)
	./$(TARGET)

## Install Python dependencies
deps:
	@echo "  [DEP] Installing Python dependencies..."
	pip3 install matplotlib numpy
	@echo "  [DEP] Done."

## Clean build artifacts
clean:
	@echo "  [CLN] Removing build artifacts..."
	rm -f $(TARGET) *.o
	@echo "  [CLN] Done."

## Help
help:
	@echo ""
	@echo "  CAN Bus 3D Vehicle Simulator — Build Targets"
	@echo "  ─────────────────────────────────────────────"
	@echo "  make           Build C++ binary"
	@echo "  make cpp       Build C++ binary"
	@echo "  make python    Run Python visualizer"
	@echo "  make run       Build + run C++ binary"
	@echo "  make deps      Install Python deps (pip)"
	@echo "  make clean     Remove build artifacts"
	@echo "  make help      Show this help"
	@echo ""
