CC      = gcc
CPLUS   = g++
AR      = ar

# Optimisation, warnings, SIMD, OpenMP
CFLAGS  = -O3 -Wall -mavx2 -mfma -fopenmp -std=c11
CXXFLAGS = -O3 -Wall -mavx2 -mfma -fopenmp -std=c++17 -DHAVE_ZLIB

_OBJS   = hull.o ch.o io.o crust.o power.o rand.o pointops.o fg.o math.o \
          predicates.o heap.o label.o io_mesh.o
OBJS    = $(patsubst %,src/%, $(_OBJS))

_HDRS   = hull.h points.h pointsites.h stormacs.h io_mesh.h simd_math.h
HDRS    = $(patsubst %,src/%, $(_HDRS))

_SRC    = hull.c ch.c io.c crust.c power.c rand.c pointops.c fg.c math.c \
          predicates.c heap.c label.c
SRC     = $(patsubst %,src/%, $(_SRC))

PROG    = powercrust
LIB     = lib$(PROG).a


all : $(PROG) simplify orient

$(OBJS) : $(HDRS)

# C++ mesh-I/O module (explicit rule)
src/io_mesh.o : src/io_mesh.cpp src/io_mesh.h src/simd_math.h
	$(CPLUS) $(CXXFLAGS) -c -o src/io_mesh.o src/io_mesh.cpp

# hullmain needs the new headers too
src/hullmain.o : src/hullmain.c $(HDRS)
	$(CC) $(CFLAGS) -c -o src/hullmain.o src/hullmain.c

$(PROG) : $(OBJS) src/hullmain.o
	mkdir -p target
	# Link with g++ so the C++ runtime from io_mesh.o is satisfied
	$(CPLUS) $(CFLAGS) $(OBJS) src/hullmain.o -o target/$(PROG) -lm -lz
	$(AR) rcv target/$(LIB) $(OBJS)

simplify : src/powershape.C src/sdefs.h
	$(CPLUS) $(CXXFLAGS) -o target/simplify src/powershape.C -lm

orient : src/setNormals.C src/ndefs.h
	$(CPLUS) $(CXXFLAGS) -o target/orient src/setNormals.C -lm

clean :
	-rm -f $(OBJS) src/hullmain.o
	-rm -rf target
