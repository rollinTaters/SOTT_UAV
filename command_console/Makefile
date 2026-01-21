CC := g++
raylib_dir := ./../raylib-5.5_linux_amd64
raylib_tablet_dir := ./../raylib-5.5_tablet_aarch64

# ==== command console objects ====
CC_OBJECTS := command_console.o console_graphics.o gauge.o user_input.o toast.o serial_manager.o joystick.o
COMMON_OBJECTS := utility.o comms_module.o video_feed.o

CC_RELEASE_OBJECTS = $(addprefix obj/release_, $(CC_OBJECTS))
CC_COMMON_OBJECTS := $(addprefix ../common_code/obj/, $(COMMON_OBJECTS))
CC_COMMON_RELEASE_OBJECTS := $(addprefix ../common_code/obj/release_, $(COMMON_OBJECTS))
CC_DEBUG_O3_OBJECTS = $(addprefix obj/debug_o3_, $(CC_OBJECTS))
CC_TABLET_OBJECTS = $(addprefix obj/tablet_, $(CC_OBJECTS))
CC_DEBUG_O0_OBJECTS = $(addprefix obj/debug_o0_, $(CC_OBJECTS))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra
DEBUG_FLAGS_O0 := -g -ggdb -O0 -Wall -Wextra
RELEASE_FLAGS := -O3 -std=c++17

DEBUG_LDFLAGS := -Wl,-rpath=$(raylib_dir)/lib -I$(raylib_dir)/include -I../common_code/src  -L$(raylib_dir)/lib -lraylib -lSDL2
LDFLAGS := -Wl,-rpath=./lib  -I$(raylib_dir)/include -I../common_code/src  -L$(raylib_dir)/lib  -lraylib -lSDL2

TABLET_LDFLAGS := -Wl,-rpath=$(raylib_tablet_dir)/lib -I$(raylib_tablet_dir)/include -I../common_code/src  -L$(raylib_tablet_dir)/lib -lraylib -lSDL2

RELEASE_PATH := ../release_builds

.PHONY: clean release src/main.hpp src/command_console.hpp tablet


# ==== Command Console Linux Debug buid ====
command_console: $(CC_DEBUG_O3_OBJECTS) $(CC_COMMON_OBJECTS)
	$(CC) $(DEBUG_FLAGS_O3) $(CC_DEBUG_O3_OBJECTS) $(CC_COMMON_OBJECTS) -o command_console $(DEBUG_LDFLAGS)

tablet: $(CC_TABLET_OBJECTS) $(CC_COMMON_OBJECTS)
	clang++ $(DEBUG_FLAGS_O3) -DTABLET_MODE $(CC_TABLET_OBJECTS) $(CC_COMMON_OBJECTS) -o command_console $(TABLET_LDFLAGS)

$(CC_COMMON_OBJECTS): ../common_code/obj/%.o: ../common_code/src/%.cpp ../common_code/src/%.hpp
	make -C ../common_code

$(CC_DEBUG_O3_OBJECTS): obj/debug_o3_%.o: src/%.cpp src/%.hpp
	$(CC) $(DEBUG_FLAGS_O3) -Wno-missing-field-initializers -c $< -o $@ $(LDFLAGS)

$(CC_TABLET_OBJECTS): obj/tablet_%.o: src/%.cpp src/%.hpp
	clang++ $(DEBUG_FLAGS_O3) -DTABLET_MODE -Wno-missing-field-initializers -c $< -o $@ $(TABLET_LDFLAGS)

clean:
	rm -f *.o obj/*.o command_console ../common_code/obj/*.o
	make command_console

# ==== Release commands ====
release:
	rm -r -f obj/release_*.o 
	make CC_linux
	#tar -C ./$(RELEASE_PATH) -acf ./$(RELEASE_PATH)/linux64_$(BUILD_VER).tar linux

# ==== Release build ====
CC_linux: $(CC_RELEASE_OBJECTS)
	$(CC) $(RELEASE_FLAGS) $(CC_RELEASE_OBJECTS) $(CC_COMMON_RELEASE_OBJECTS) -o $(RELEASE_PATH)/CC_linux $(LDFLAGS)

$(CC_RELEASE_OBJECTS): obj/release_%.o : src/%.cpp
	$(CC) $(RELEASE_FLAGS) -c $< -o $@ $(LDFLAGS)
