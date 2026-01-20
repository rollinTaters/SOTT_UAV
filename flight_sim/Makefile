CC := g++
raylib_dir := ./../raylib-5.5_linux_amd64
raylib_tablet_dir := ./../raylib-5.5_tablet_aarch64

# ==== command console objects ====
FS_OBJECTS := sim_engine.o joystick.o
COMMON_OBJECTS := utility.o comms_module.o video_feed.o

FS_RELEASE_OBJECTS = $(addprefix obj/linux_, $(FS_OBJECTS))
COMMON_OBJECTS := $(addprefix ../common_code/obj/, $(COMMON_OBJECTS))
FS_DEBUG_O3_OBJECTS = $(addprefix obj/debug_o3_, $(FS_OBJECTS))
FS_TABLET_OBJECTS = $(addprefix obj/tablet_, $(FS_OBJECTS))
FS_DEBUG_O0_OBJECTS = $(addprefix obj/debug_o0_, $(FS_OBJECTS))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra
DEBUG_FLAGS_O0 := -g -ggdb -O0 -Wall -Wextra
RELEASE_FLAGS := -O3 -std=c++17

DEBUG_LDFLAGS := -Wl,-rpath=$(raylib_dir)/lib -I $(raylib_dir)/include  -L$(raylib_dir)/lib -lraylib -lSDL2
LDFLAGS := -Wl,-rpath=./lib  -I $(raylib_dir)/include  -L$(raylib_dir)/lib  -lraylib -lSDL2

TABLET_LDFLAGS := -Wl,-rpath=$(raylib_tablet_dir)/lib -I $(raylib_tablet_dir)/include  -L$(raylib_tablet_dir)/lib -lraylib -lSDL2


BUILD_VER := 0.2
RELEASE_PATH := release_builds/v$(BUILD_VER)

.PHONY: clean release src/main.hpp src/sim_engine.hpp tablet


# ==== Command Console Linux Debug buid ====
flight_sim: $(FS_DEBUG_O3_OBJECTS) $(COMMON_OBJECTS)
	$(CC) $(DEBUG_FLAGS_O3) $(FS_DEBUG_O3_OBJECTS) $(COMMON_OBJECTS) -o flight_sim $(DEBUG_LDFLAGS)

tablet: $(FS_TABLET_OBJECTS) $(COMMON_OBJECTS)
	clang++ $(DEBUG_FLAGS_O3) -DTABLET_MODE $(FS_TABLET_OBJECTS) $(COMMON_OBJECTS) -o flight_sim $(TABLET_LDFLAGS)

$(COMMON_OBJECTS): ../common_code/obj/%.o: ../common_code/src/%.cpp ../common_code/src/%.hpp
	make -C ../common_code

$(FS_DEBUG_O3_OBJECTS): obj/debug_o3_%.o: src/%.cpp src/%.hpp
	$(CC) $(DEBUG_FLAGS_O3) -Wno-missing-field-initializers -c $< -o $@ $(LDFLAGS)

$(FS_TABLET_OBJECTS): obj/tablet_%.o: src/%.cpp src/%.hpp
	clang++ $(DEBUG_FLAGS_O3) -DTABLET_MODE -Wno-missing-field-initializers -c $< -o $@ $(TABLET_LDFLAGS)

clean:
	rm -f *.o obj/*.o flight_sim ../common_code/obj/*.o
	make flight_sim

# ==== Release commands ====
release:
	rm -r -f $(RELEASE_PATH) obj/linux_*.o 
	make $(RELEASE_PATH)
	make FS_linux
	tar -C ./$(RELEASE_PATH) -acf ./$(RELEASE_PATH)/linux64_$(BUILD_VER).tar linux

$(RELEASE_PATH):
	mkdir $(RELEASE_PATH)
	mkdir $(RELEASE_PATH)/linux
	mkdir $(RELEASE_PATH)/linux/flight_sim

# ==== Release build ====
FS_linux: $(FS_RELEASE_OBJECTS)
	cp -r gfx $(RELEASE_PATH)/linux/flight_sim/gfx
	$(CC) $(RELEASE_FLAGS) $(FS_RELEASE_OBJECTS) -o $(RELEASE_PATH)/linux/flight_sim/FS_linux $(LDFLAGS)

$(FS_RELEASE_OBJECTS): obj/linux_%.o : src/%.cpp
	$(CC) $(RELEASE_FLAGS) -c $< -o $@ $(LDFLAGS)
