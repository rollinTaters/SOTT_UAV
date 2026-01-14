CC := g++
raylib_dir := ./../raylib-5.5_linux_amd64

# ==== command console objects ====
CC_OBJECTS := command_console.o console_graphics.o gauge.o user_input.o toast.o 
COMMON_OBJECTS := utility.o comms_module.o video_feed.o

CC_RELEASE_OBJECTS = $(addprefix obj/linux_, $(CC_OBJECTS))
COMMON_OBJECTS := $(addprefix ../common_code/obj/, $(COMMON_OBJECTS))
CC_DEBUG_O3_OBJECTS = $(addprefix obj/debug_o3_, $(CC_OBJECTS))
CC_DEBUG_O0_OBJECTS = $(addprefix obj/debug_o0_, $(CC_OBJECTS))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra
DEBUG_FLAGS_O0 := -g -ggdb -O0 -Wall -Wextra
RELEASE_FLAGS := -O3 -std=c++17

DEBUG_LDFLAGS := -Wl,-rpath=$(raylib_dir)/lib -I $(raylib_dir)/include  -L$(raylib_dir)/lib -lraylib
LDFLAGS := -Wl,-rpath=./lib  -I $(raylib_dir)/include  -L$(raylib_dir)/lib  -lraylib


BUILD_VER := 0.2
RELEASE_PATH := release_builds/v$(BUILD_VER)

.PHONY: clean release src/main.hpp src/command_console.hpp


# ==== Command Console Linux Debug buid ====
command_console: $(CC_DEBUG_O3_OBJECTS) $(COMMON_OBJECTS)
	$(CC) $(DEBUG_FLAGS_O3) $(CC_DEBUG_O3_OBJECTS) $(COMMON_OBJECTS) -o command_console $(DEBUG_LDFLAGS)

$(COMMON_OBJECTS): ../common_code/obj/%.o: ../common_code/src/%.cpp ../common_code/src/%.hpp
	make -C ../common_code

$(CC_DEBUG_O3_OBJECTS): obj/debug_o3_%.o: src/%.cpp src/%.hpp
	$(CC) $(DEBUG_FLAGS_O3) -Wno-missing-field-initializers -c $< -o $@ $(LDFLAGS)

clean:
	rm -f *.o obj/*.o command_console ../common_code/obj/*.o
	make command_console

# ==== Release commands ====
release:
	rm -r -f $(RELEASE_PATH) obj/linux_*.o 
	make $(RELEASE_PATH)
	make CC_linux
	tar -C ./$(RELEASE_PATH) -acf ./$(RELEASE_PATH)/linux64_$(BUILD_VER).tar linux

$(RELEASE_PATH):
	mkdir $(RELEASE_PATH)
	mkdir $(RELEASE_PATH)/linux
	mkdir $(RELEASE_PATH)/linux/cc

# ==== Release build ====
CC_linux: $(CC_RELEASE_OBJECTS)
	cp -r gfx $(RELEASE_PATH)/linux/cc/gfx
	$(CC) $(RELEASE_FLAGS) $(CC_RELEASE_OBJECTS) -o $(RELEASE_PATH)/linux/cc/CC_linux $(LDFLAGS)

$(CC_RELEASE_OBJECTS): obj/linux_%.o : src/%.cpp
	$(CC) $(RELEASE_FLAGS) -c $< -o $@ $(LDFLAGS)
