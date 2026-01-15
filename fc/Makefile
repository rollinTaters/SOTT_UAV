# --- Configuration ---
PORT = /dev/ttyUSB0
FQBN = arduino:avr:nano
SKETCH = MyProject.ino

# Path to a specific external library folder
# (Point this to the folder containing the library's library.properties or src/)
EXTRA_LIB = /home/user/path/to/my_shared_lib

# If you have multiple libraries, you can use a directory containing many of them
LIB_DIR = /home/user/common_arduino_libs

# --- Rules ---
all: compile upload

compile:
	./arduino-cli compile --fqbn $(FQBN) \
		--build-property "compiler.cpp.extra_flags=-I/../common_code/src" \
		--library $(EXTRA_LIB) \
		--libraries $(LIB_DIR) \
		$(SKETCH)

upload:
	./arduino-cli upload -p $(PORT) --fqbn $(FQBN) $(SKETCH)

# Helper to install a new library from the official registry
install-deps:
	#./arduino-cli lib install "DHT sensor library"
	#./arduino-cli lib install "Adafruit Unified Sensor"

clean:
	rm -rf build/

monitor:
	./arduino-cli monitor -p $(PORT)
