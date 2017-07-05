# smoothener
Convert multi-robot waypoint sequences into smooth piecewise polynomial trajectories.

This repository contains an implementation of the continuous trajectory optimization stage
of the algorithm described in:

> *Downwash-Aware Trajectory Planning for Large Quadcopter Teams*
>
> James A. Preiss, Wolfgang Hönig, Nora Ayanian, Gaurav S. Sukhatme
>
> Accepted at IEEE IROS 2017,
> Preprint at [https://arxiv.org/abs/1704.04852](https://arxiv.org/abs/1704.04852)


## overview

The purpose of this program is to convert a waypoint sequence for multiple robots
into a set of smooth trajectories.
It is assumed that the waypoint sequence comes from a planner (typically graph-based)
that models robots moving on straight lines between waypoints.
It is impossible to execute such a plan in real life with nonzero velocity at the waypoints,
because it would require infinite acceleration.
Therefore, we want to "smoothen" the trajectories while maintaining
the collision avoidance guarantee.

We model robots as axis-aligned ellipsoids to deal with the downwash effect of quadrotors.
The output format is piecewise polynomial trajectories with user-specified smoothness and degree.
Output is given as Matlab's [ppform](https://www.mathworks.com/help/curvefit/the-ppform.html) structs.


## setup instructions
1. Make sure your Matlab MEX compiler is set up correctly, and uses at least -O2 optimization.
2. Run `make`.
3. From the `smoothener` root directory , open a Matlab session and run `main_grid`.
   Computation should take several seconds, and you should see a 3D plot when it is done.

### extra setup to use Octomap:
4. run `git submodule init && git submodule update`.
5. `cd` into `octomap_corridor/octomap` and follow the "to only compile the library" instructions in Octomap's `README.md`.
6. `cd` back into the `smoothener` root directory and run `make octomap`.
7. From the `smoothener` root directory , open a Matlab session and run `main_octomap`.
   Computation should take several seconds, and you should see a 3D plot when it is done.


## environment obstacles

Currently supported environment models are:

* [Octomap](https://octomap.github.io/)s, given as binary files on the disk
* sets of axis-aligned boxes

This part of the code is modularized, making it easy to add support for other environment representations.


## project structure

item | description
---- | -----------
`<top level>` | main routines, obstacle model implementations, and makefile. The main entry point is `smoothener.m`.
`examples/` | example problems for both "octomap" and "list of boxes" environment models, and an obstacle-free problem.
`external/` | third-party code from the Matlab file exchange.
`octomap_corridor/` | a standalone program that computes the safe corridor for a single robot in an octomap. A separate process is used instead of a mex-function because the auto-generated SVM solver C code from CVXGEN uses global variables, making multithreading impossible.
`svm_sephyp_*point/` | generated C code from the CVXGEN package to solve ellipsoid-weighted hard margin SVM problems for separating hyperplanes in 3D. Used for both robot-robot and robot-obstacle separation. Also contains mex-functions implementing the outer loop for all robots.
`tests/` | very few unit tests, need to write some more...
`utils/` | low-level simple subroutines.

   
## project status

### major TODOs:
- Support 2D problems (most "difficult" parts of the code are dimension-generic, but some places assume 3D,
  particularly the CVXGEN generated code for separating hyperplane problems.)


## implementation notes

The implementation uses generated code from
[CVXGEN](https://cvxgen.com/docs/index.html) to solve many small separating-hyperplane problems.
This is necessary to avoid excessive overhead of interpreting Matlab code.
The makefile compiles all C/C++ code.

The implementation has been profiled and optimized.
There is no known low-hanging fruit.
For medium-sized problems (20-80 robots), the bottleneck is solving the large quadratic programs
for each robot's corridor-constrained trajectory optimization problem.
If [CPLEX](https://www-01.ibm.com/software/commerce/optimization/cplex-optimizer/) is installed,
it will be used to solve these QPs.

Wherever possible, computational bottlenecks are parallelized.
Multiple CPU cores up to the number of robots will show a large benefit.

