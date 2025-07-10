# Justfile for CAP1188 Library
# Provides convenient build and flash commands for Raspberry Pi Pico

# Default recipe - show available commands
default:
    @just --list

# Setup clangd configuration for ARM cross-compilation
setup-clangd:
    @echo "Setting up clangd for ARM cross-compilation..."
    @./scripts/setup-clangd.sh

# Build the project
build:
    @echo "Building CAP1188 library and examples..."
    @mkdir -p build
    @cd build && cmake ..
    @cd build && make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
    @echo "Build complete! Files in build/ directory:"
    @ls -la build/*.uf2 2>/dev/null || echo "No .uf2 files found"

# Clean build directory
clean:
    @echo "Cleaning build directory..."
    @rm -rf build/
    @echo "Clean complete!"

# Clean everything (build and docs)
clean-all: clean clean-docs
    @echo "Complete cleanup finished!"

# Build and flash the basic touch example
flash: build
    @echo "Flashing basic_touch example to Pico..."
    @if [ ! -f "build/basic_touch.uf2" ]; then \
        echo "Error: basic_touch.uf2 not found in build directory"; \
        exit 1; \
    fi
    @echo "Put your Pico in BOOTSEL mode (hold BOOTSEL while connecting USB)"
    @echo "Press Enter when ready..."
    @read
    @picotool load build/basic_touch.uf2 --force
    @echo "Flash complete! Check serial output for touch detection."


# Development build with verbose output
dev-build:
    @echo "Development build with verbose output..."
    @mkdir -p build
    @cd build && cmake -DCMAKE_BUILD_TYPE=Debug ..
    @cd build && make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4) VERBOSE=1

# Release build (optimized)
release-build:
    @echo "Release build (optimized)..."
    @mkdir -p build
    @cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
    @cd build && make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)

# Monitor serial output from the Pico
monitor:
    @echo "Monitoring serial output from Pico..."
    @echo "Press Ctrl+A then Ctrl+X to exit"
    @picocom -b 115200 --imap lfcrlf /dev/cu.usbmodem*

# Test build: build, flash, reboot, and monitor
test-build: build
    @echo "Test build: flashing and monitoring basic_touch example..."
    @if [ ! -f "build/basic_touch.uf2" ]; then \
        echo "Error: basic_touch.uf2 not found in build directory"; \
        exit 1; \
    fi
    @echo "Put your Pico in BOOTSEL mode (hold BOOTSEL while connecting USB)"
    @echo "Press Enter when ready..."
    @read
    @picotool load build/basic_touch.uf2 --force
    @echo "Flashing complete! Rebooting Pico..."
    @sleep 2
    @echo "Starting serial monitor..."
    @echo "Press Ctrl+A then Ctrl+X to exit monitoring"
    @picocom -b 115200 --imap lfcrlf /dev/cu.usbmodem*

# Generate documentation
docs:
    @echo "Generating API documentation..."
    @if ! command -v doxygen >/dev/null 2>&1; then \
        echo "Error: Doxygen not found. Please install doxygen to generate documentation."; \
        echo "macOS: brew install doxygen"; \
        echo "Ubuntu/Debian: sudo apt-get install doxygen"; \
        echo "Other: See https://www.doxygen.nl/download.html"; \
        exit 1; \
    fi
    @mkdir -p docs/generated
    @doxygen Doxyfile
    @echo "Documentation generated in docs/generated/html/"
    @echo "Open docs/generated/html/index.html in your browser to view"

# Build documentation with cmake
docs-cmake: build
    @echo "Building documentation with CMake..."
    @cd build && make docs
    @echo "Documentation generated in build/docs/generated/html/"

# Open documentation in browser (macOS)
open-docs: docs
    @if [ -f "docs/generated/html/index.html" ]; then \
        open docs/generated/html/index.html; \
    else \
        echo "Documentation not found. Run 'just docs' first."; \
    fi

# Clean documentation
clean-docs:
    @echo "Cleaning documentation..."
    @rm -rf docs/generated
    @echo "Documentation cleaned!"
