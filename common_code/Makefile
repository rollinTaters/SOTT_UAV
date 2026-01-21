# ==== Common Code objects ====
OBJ2COMPILE := utility.o comms_module.o video_feed.o

COMMON_OBJECTS = $(addprefix obj/, $(OBJ2COMPILE))
RELEASE_OBJECTS = $(addprefix obj/release_, $(OBJ2COMPILE))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra
RELEASE_FLAGS := -O3 -std=c++17

#DEBUG_LDFLAGS := no flags
LDFLAGS := -Wl,-rpath=./lib

COMPILER := g++

.PHONY: clean all release

# ==== Unmanned Land Vehicle Linux Debug build ====
all: $(COMMON_OBJECTS)

$(COMMON_OBJECTS): obj/%.o: src/%.cpp src/%.hpp
	$(COMPILER) $(DEBUG_FLAGS_O3) -c $< -o $@ $(LDFLAGS)
	
$(RELEASE_OBJECTS): obj/release_%.o : src/%.cpp
	$(CC) $(RELEASE_FLAGS) -c $< -o $@ $(LDFLAGS)
	
clean:
	rm -f *.o obj/*.o
	make
	
release: $(RELEASE_OBJECTS)





