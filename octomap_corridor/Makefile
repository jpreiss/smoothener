CXXFLAGS = -std=c++11 -Ioctomap/octomap/include -O3
CFLAGS = -O3 -x c

CC = $(CXX)

octomap_corridor : octomap_corridor.o \
octomap/octomap/lib/liboctomap.a octomap/octomap/lib/liboctomath.a \
cvxgen/ldl.o cvxgen/matrix_support.o cvxgen/solver.o cvxgen/util.o

clean:
	rm -rf cvxgen/*.o
	rm -rf octomap_corridor *.o
