# ==== Common Code objects ====
OBJ2COMPILE := utility.o comms_module.o video_feed.o

COMMON_OBJECTS = $(addprefix obj/, $(OBJ2COMPILE))


# ==== flags ====
DEBUG_FLAGS_O3 := -g -ggdb -O3 -Wall -Wextra

#DEBUG_LDFLAGS := no flags
LDFLAGS := -Wl,-rpath=./lib

COMPILER := g++

.PHONY: clean all

# ==== Unmanned Land Vehicle Linux Debug build ====
all: $(COMMON_OBJECTS)

$(COMMON_OBJECTS): obj/%.o: src/%.cpp src/%.hpp
	$(COMPILER) $(DEBUG_FLAGS_O3) -c $< -o $@ $(LDFLAGS)

clean:
	rm -f *.o obj/*.o
	make




