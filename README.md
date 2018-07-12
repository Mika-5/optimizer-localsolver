optimizer-localsolver
===================

Compute an optimized solution to the Vehicle Routing Problem using localsolver.

Installation
============

## Optimizer

Require localsolver.

Compile the C++ optimizer

    make third_party
    make tsp_localsolver

Test
====

LD_LIBRARY_PATH=../optimizer-localsolver/dependencies/install/lib/:../localsolver_8_0/include/ ../optimizer-localsolver/tsp_localsolver  -instance_file 'instance_licent_localsolver' -solution_file '/tmp/optimize-localsolver-output20180712-13097-xp90cp'
