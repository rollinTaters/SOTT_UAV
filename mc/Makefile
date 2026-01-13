raylib_dir := ./../raylib-5.5_linux_amd64
raylib_x32_dir := ./../raylib-5.5_linux_i386
opencv_dir := /usr/local/include/opencv4

# ==== Navigation Guidance and Control objects ====
MC_OBJECTS := main.o
COMMON_OBJECTS := utility.o comms_module.o video_feed.o

MC_RELEASE_OBJECTS = $(addprefix obj/, $(MC_OBJECTS))
MC_COMMON_OBJECTS = $(addprefix ../common_code/obj/, $(COMMON_OBJECTS))
MC_DEBUG_O3_OBJECTS = $(addprefix obj/debug_o3_, $(MC_OBJECTS))
MC_DEBUG_GUI_OBJECTS = $(addprefix obj/debug_gui_, $(MC_OBJECTS))
MC_PI_OBJECTS = $(addprefix obj/pi_, $(MC_OBJECTS))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra
DEBUG_FLAGS_O0 := -g -ggdb -O0 -Wall -Wextra
RELEASE_FLAGS := -O3 -std=c++17

DEBUG_LDFLAGS := -Wl,-rpath=$(raylib_dir)/lib -I $(raylib_dir)/include -I $(opencv_dir) -L$(raylib_dir)/lib -lraylib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lopencv_highgui
LDFLAGS := -Wl,-rpath=./lib -I $(raylib_dir)/include -L$(raylib_dir)/lib -lraylib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio
PI_LDFLAGS := -Wl,-rpath=./lib -I $(raylib_x32_dir)/include -L$(raylib_x32_dir)/lib -lraylib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio

COMPILER := g++

BUILD_VER := 0.2
RELEASE_PATH := release_builds/v$(BUILD_VER)
PI_TEST_PATH := ../pi_test

.PHONY: clean clear release gui headless src/main.hpp

# ==== Unmanned Land Vehicle Linux Debug build ====
gui: $(MC_DEBUG_GUI_OBJECTS) $(MC_COMMON_OBJECTS)
	$(COMPILER) $(DEBUG_FLAGS_O3) -DDEBUG_GUI $(MC_DEBUG_GUI_OBJECTS) $(MC_COMMON_OBJECTS) -o mc_gui $(DEBUG_LDFLAGS)
	
pi: $(MC_PI_OBJECTS) $(COMMON_PI_OBJECTS) $(PI_TEST_PATH)
	$(COMPILER) $(RELEASE_FLAGS) $(MC_PI_OBJECTS) $(COMMON_PI_OBJECTS) -o $(PI_TEST_PATH)/mc_pi $(PI_LDFLAGS)

headless: $(MC_DEBUG_O3_OBJECTS) $(MC_COMMON_OBJECTS)
	$(COMPILER) $(DEBUG_FLAGS_O3) $(MC_DEBUG_O3_OBJECTS) $(MC_COMMON_OBJECTS) -o mc $(DEBUG_LDFLAGS)

$(MC_COMMON_OBJECTS): ../common_code/obj/%.o: ../common_code/src/%.cpp ../common_code/src/%.hpp
	make -C ../common_code
	
$(COMMON_PI_OBJECTS): obj/common_pi_%.o: ../common_code/src/%.cpp ../common_code/src/%.hpp
	$(COMPILER) $(RELEASE_FLAGS) -c $< -o $@ $(PI_LDFLAGS)
	
$(MC_PI_OBJECTS): obj/pi_%.o: src/%.cpp src/%.hpp
	$(COMPILER) $(RELEASE_FLAGS) -c $< -o $@ $(PI_LDFLAGS)

$(MC_DEBUG_O3_OBJECTS): obj/debug_o3_%.o: src/%.cpp src/%.hpp
	$(COMPILER) $(DEBUG_FLAGS_O3) -c $< -o $@ $(DEBUG_LDFLAGS)

$(MC_DEBUG_GUI_OBJECTS): obj/debug_gui_%.o: src/%.cpp src/%.hpp
	$(COMPILER) $(DEBUG_FLAGS_O0) -Wno-missing-field-initializers -DDEBUG_GUI -c $< -o $@ $(DEBUG_LDFLAGS)

clean:
	rm -f *.o obj/*.o mc ../common_code/obj/*.o
	make 

clear:
	rm -f *.o obj/*.o mc ../common_code/obj/*.o


# ==== Release commands ====
release:
	rm -r -f $(RELEASE_PATH) obj/*.o
	make $(RELEASE_PATH)
	make MC_linux
	tar -C ./$(RELEASE_PATH) -acf ./$(RELEASE_PATH)/linux64_$(BUILD_VER).tar MC_linux

$(PI_TEST_PATH):
	mkdir $(PI_TEST_PATH)
	mkdir $(PI_TEST_PATH)/lib
	cp -r $(raylib_x32_dir)/lib $(PI_TEST_PATH)/lib

$(RELEASE_PATH):
	mkdir $(RELEASE_PATH)
	#mkdir $(RELEASE_PATH)/win
	#mkdir $(RELEASE_PATH)/win/mc
	mkdir $(RELEASE_PATH)/linux
	mkdir $(RELEASE_PATH)/linux/mc

# ==== Linux Release build ====
MC_linux: $(MC_RELEASE_OBJECTS)
	#cp -r gfx $(RELEASE_PATH)/linux/mc/gfx
	$(COMPILER) $(RELEASE_FLAGS) $(MC_RELEASE_OBJECTS) -o $(RELEASE_PATH)/linux/mc/MC_linux $(LDFLAGS)

$(MC_RELEASE_OBJECTS): obj/%.o : src/%.cpp
	$(COMPILER) $(RELEASE_FLAGS) -c $< -o $@ $(LDFLAGS)




