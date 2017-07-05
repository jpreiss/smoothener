% compiles the optional mex functions that use Octomap. "make octomap" runs this for you.

cd utils
mex CXXFLAGS='$CXXFLAGS -I../octomap_corridor/octomap/octomap/include -std=c++11' read_octomap_bbox_mex.cpp ../octomap_corridor/octomap/octomap/lib/liboctomap.a ../octomap_corridor/octomap/octomap/lib/liboctomath.a
cd ..
