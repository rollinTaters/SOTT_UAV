SFML_dir_linux := ./../SFML/SFML-2.6.x-linux
SFML_dir_win := ./../SFML/SFML-2.6.x-win

SFML_libs := -lsfml-audio -lsfml-graphics -lsfml-window -lsfml-system -lsfml-network
#SFML_libs_static := -DSFML_STATIC -MMD -MP -static-libstdc++ -static-libgcc -lsfml-main -lsfml-audio-s -lsfml-graphics-s -lsfml-window-s -lsfml-system-s -lsfml-network-s  -lopengl32 -lfreetype -lwinmm -lgdi32 -lopenal32 -lFLAC -lvorbisenc -lvorbis -logg -lws2_32

# ==== Common Code objects ====
COMMON_OBJECTS := utility.o comms_module.o video_feed.o

LINUX_COMMON_OBJECTS = $(addprefix obj/linux_, $(COMMON_OBJECTS))
WIN_COMMON_OBJECTS = $(addprefix obj/win_, $(COMMON_OBJECTS))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra

LINUX_DEBUG_LDFLAGS := -Wl,-rpath=$(SFML_dir_linux)/lib -I $(SFML_dir_linux)/include -L$(SFML_dir_linux)/lib $(SFML_libs)
LINUX_LDFLAGS := -Wl,-rpath=./lib -I $(SFML_dir_linux)/include -L$(SFML_dir_linux)/lib $(SFML_libs)
WIN_LDFLAGS := -mwindows -I $(SFML_dir_win)/include -L$(SFML_dir_win)/lib -L/usr/x86_64-w64-mingw32/lib -L/usr/x86_64-w64-mingw32/bin $(SFML_libs) -lsfml-main

WIN_COMPILER := x86_64-w64-mingw32-g++-posix
LINUX_COMPILER := g++

.PHONY: clean all

# ==== Unmanned Land Vehicle Linux Debug build ====
all: $(LINUX_COMMON_OBJECTS)

$(LINUX_COMMON_OBJECTS): obj/linux_%.o: src/%.cpp src/%.hpp
	$(LINUX_COMPILER) $(DEBUG_FLAGS_O3) -c $< -o $@ $(LINUX_LDFLAGS)

$(WIN_COMMON_OBJECTS): obj/win_%.o: src/%.cpp src/%.hpp
	$(WIN_COMPILER) $(DEBUG_FLAGS_O3) -c $< -o $@ $(WIN_LDFLAGS)

clean:
	rm -f *.o obj/*.o
	make




