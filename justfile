# Justfile for CAP1188 Library
# Provides convenient build and flash commands for Raspberry Pi Pico

# Default recipe - show available commands
default:
    @just --list

# Build the project
build:
    @echo "Building CAP1188 library and examples..."
    @mkdir -p build
    @cd build && cmake ..
    @cd build && make -j$(nproc)
    @echo "Build complete! Files in build/ directory:"
    @ls -la build/*.uf2 2>/dev/null || echo "No .uf2 files found"

# Clean build directory
clean:
    @echo "Cleaning build directory..."
    @rm -rf build/
    @echo "Clean complete!"

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

# Flash using drag-and-drop method (when picotool is not available)
flash-manual: build
    @echo "Manual flash instructions:"
    @echo "1. Hold BOOTSEL button while connecting Pico to USB"
    @echo "2. Copy build/basic_touch.uf2 to the Pico drive"
    @echo "3. The Pico will automatically reboot and run the program"
    @if [ -f "build/basic_touch.uf2" ]; then \
        echo "File ready: build/basic_touch.uf2"; \
    else \
        echo "Error: basic_touch.uf2 not found!"; \
    fi


# Check if picotool is available
check-tools:
    @echo "Checking required tools..."
    @if command -v cmake >/dev/null 2>&1; then \
        echo "✓ CMake found: $(cmake --version | head -1)"; \
    else \
        echo "✗ CMake not found - please install cmake"; \
    fi
    @if command -v make >/dev/null 2>&1; then \
        echo "✓ Make found: $(make --version | head -1)"; \
    else \
        echo "✗ Make not found - please install build-essential"; \
    fi
    @if command -v picotool >/dev/null 2>&1; then \
        echo "✓ Picotool found: $(picotool version)"; \
    else \
        echo "✗ Picotool not found - install from https://github.com/raspberrypi/picotool"; \
        echo "  Alternative: use 'just flash-manual' for drag-and-drop flashing"; \
    fi
    @if [ -n "$$PICO_SDK_PATH" ]; then \
        echo "✓ PICO_SDK_PATH set: $$PICO_SDK_PATH"; \
    else \
        echo "✗ PICO_SDK_PATH not set - please set environment variable"; \
    fi


# Development build with verbose output
dev-build:
    @echo "Development build with verbose output..."
    @mkdir -p build
    @cd build && cmake -DCMAKE_BUILD_TYPE=Debug ..
    @cd build && make -j$(nproc) VERBOSE=1

# Release build (optimized)
release-build:
    @echo "Release build (optimized)..."
    @mkdir -p build
    @cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
    @cd build && make -j$(nproc)

# Create a new example project
new-example name:
    @echo "Creating new example: {{name}}"
    @mkdir -p examples/{{name}}
    @echo "#include \"cap1188/cap1188.hpp\"" > examples/{{name}}/{{name}}.cpp
    @echo "" >> examples/{{name}}/{{name}}.cpp
    @echo "int main() {" >> examples/{{name}}/{{name}}.cpp
    @echo "    // Your code here" >> examples/{{name}}/{{name}}.cpp
    @echo "    return 0;" >> examples/{{name}}/{{name}}.cpp
    @echo "}" >> examples/{{name}}/{{name}}.cpp
    @echo "Example created: examples/{{name}}/{{name}}.cpp"
    @echo "Add build target to CMakeLists.txt to compile it."