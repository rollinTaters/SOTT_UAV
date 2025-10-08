SFML_dir := ./../SFML/SFML-2.6.x-linux
raylib_dir := ./../raylib-5.5_linux_amd64
raylib_x32_dir := ./../raylib-5.5_linux_i386

#SFML_libs := -lsfml-audio -lsfml-graphics -lsfml-window -lsfml-system -lsfml-network
#SFML_libs_static := -DSFML_STATIC -MMD -MP -static-libstdc++ -static-libgcc -lsfml-main -lsfml-audio-s -lsfml-graphics-s -lsfml-window-s -lsfml-system-s -lsfml-network-s  -lopengl32 -lfreetype -lwinmm -lgdi32 -lopenal32 -lFLAC -lvorbisenc -lvorbis -logg -lws2_32

# ==== Navigation Guidance and Control objects ====
NGC_OBJECTS := main.o vehicle.o ngc.o sensor_emulator.o env_emulator.o traction_motor.o
COMMON_OBJECTS := utility.o comms_module.o video_feed.o

LINUX_NGC_RELEASE_OBJECTS = $(addprefix obj/linux_, $(NGC_OBJECTS))
LINUX_COMMON_OBJECTS = $(addprefix ../common_code/obj/linux_, $(COMMON_OBJECTS))
LINUX_NGC_DEBUG_O3_OBJECTS = $(addprefix obj/debug_o3_, $(NGC_OBJECTS))
LINUX_NGC_DEBUG_GUI_OBJECTS = $(addprefix obj/debug_gui_, $(NGC_OBJECTS))
LINUX_NGC_PI_OBJECTS = $(addprefix obj/pi_, $(NGC_OBJECTS))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra
DEBUG_FLAGS_O0 := -g -ggdb -O0 -Wall -Wextra
RELEASE_FLAGS := -O3 -std=c++17

LINUX_DEBUG_LDFLAGS := -Wl,-rpath=$(raylib_dir)/lib:$(SFML_dir)/lib -I $(raylib_dir)/include -I $(SFML_dir)/include -L$(raylib_dir)/lib -L$(SFML_dir)/lib -lraylib -lsfml-system -lsfml-network
LINUX_LDFLAGS := -Wl,-rpath=./lib -I $(raylib_dir)/include -L$(raylib_dir)/lib -lraylib
LINUX_PI_LDFLAGS := -Wl,-rpath=./lib -I $(raylib_x32_dir)/include -L$(raylib_x32_dir)/lib -lraylib

LINUX_COMPILER := g++

BUILD_VER := 0.2
RELEASE_PATH := release_builds/v$(BUILD_VER)
PI_TEST_PATH := ../pi_test

.PHONY: clean clear release gui headless src/main.hpp

# ==== Unmanned Land Vehicle Linux Debug build ====
gui: $(LINUX_NGC_DEBUG_GUI_OBJECTS) $(LINUX_COMMON_OBJECTS)
	$(LINUX_COMPILER) $(DEBUG_FLAGS_O3) -DDEBUG_GUI $(LINUX_NGC_DEBUG_GUI_OBJECTS) $(LINUX_COMMON_OBJECTS) -o ngc $(LINUX_DEBUG_LDFLAGS)
	
pi: $(LINUX_NGC_PI_OBJECTS) $(LINUX_COMMON_PI_OBJECTS) $(PI_TEST_PATH)
	$(LINUX_COMPILER) $(RELEASE_FLAGS) $(LINUX_NGC_PI_OBJECTS) $(LINUX_COMMON_PI_OBJECTS) -o $(PI_TEST_PATH)/ngc_pi $(LINUX_PI_LDFLAGS)

#headless: $(LINUX_NGC_DEBUG_O3_OBJECTS) $(LINUX_COMMON_OBJECTS)
	#$(LINUX_COMPILER) $(DEBUG_FLAGS_O3) $(LINUX_NGC_DEBUG_O3_OBJECTS) $(LINUX_COMMON_OBJECTS) -o ngc $(LINUX_DEBUG_LDFLAGS)

$(LINUX_COMMON_OBJECTS): ../common_code/obj/linux_%.o: ../common_code/src/%.cpp ../common_code/src/%.hpp
	make -C ../common_code
	
$(LINUX_COMMON_PI_OBJECTS): obj/common_pi_%.o: ../common_code/src/%.cpp ../common_code/src/%.hpp
	$(LINUX_COMPILER) $(RELEASE_FLAGS) -c $< -o $@ $(LINUX_PI_LDFLAGS)
	
$(LINUX_NGC_PI_OBJECTS): obj/pi_%.o: src/%.cpp src/%.hpp
	$(LINUX_COMPILER) $(RELEASE_FLAGS) -c $< -o $@ $(LINUX_PI_LDFLAGS)

$(LINUX_NGC_DEBUG_O3_OBJECTS): obj/debug_o3_%.o: src/%.cpp src/%.hpp
	$(LINUX_COMPILER) $(DEBUG_FLAGS_O3) -c $< -o $@ $(LINUX_DEBUG_LDFLAGS)

$(LINUX_NGC_DEBUG_GUI_OBJECTS): obj/debug_gui_%.o: src/%.cpp src/%.hpp
	$(LINUX_COMPILER) $(DEBUG_FLAGS_O0) -Wno-missing-field-initializers -DDEBUG_GUI -c $< -o $@ $(LINUX_DEBUG_LDFLAGS)

clean:
	rm -f *.o obj/*.o ngc ../common_code/obj/*.o
	make 

clear:
	rm -f *.o obj/*.o ngc ../common_code/obj/*.o


# ==== Release commands ====
release:
	rm -r -f $(RELEASE_PATH) obj/linux_*.o obj/win_*.o
	make $(RELEASE_PATH)
	make NGC_linux
	#make NGC_win64
	tar -C ./$(RELEASE_PATH) -acf ./$(RELEASE_PATH)/linux64_$(BUILD_VER).tar linux
	#zip -r -q ./$(RELEASE_PATH)/win64_$(BUILD_VER).zip ./$(RELEASE_PATH)/win

$(PI_TEST_PATH):
	mkdir $(PI_TEST_PATH)
	mkdir $(PI_TEST_PATH)/lib
	cp -r $(raylib_x32_dir)/lib $(PI_TEST_PATH)/lib

$(RELEASE_PATH):
	mkdir $(RELEASE_PATH)
	#mkdir $(RELEASE_PATH)/win
	#mkdir $(RELEASE_PATH)/win/ngc
	mkdir $(RELEASE_PATH)/linux
	mkdir $(RELEASE_PATH)/linux/ngc

# ==== Linux Release build ====
NGC_linux: $(LINUX_NGC_RELEASE_OBJECTS)
	cp -r $(SFML_dir_linux)/lib $(RELEASE_PATH)/linux/ngc/lib
	#cp -r gfx $(RELEASE_PATH)/linux/ngc/gfx
	$(LINUX_COMPILER) $(RELEASE_FLAGS) $(LINUX_NGC_RELEASE_OBJECTS) -o $(RELEASE_PATH)/linux/ngc/NGC_linux $(LINUX_LDFLAGS)

$(LINUX_NGC_RELEASE_OBJECTS): obj/linux_%.o : src/%.cpp
	$(LINUX_COMPILER) $(RELEASE_FLAGS) -c $< -o $@ $(LINUX_LDFLAGS)




