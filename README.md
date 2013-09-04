sot-core
========

[![Build Status](https://travis-ci.org/stack-of-tasks/sot-core.png?branch=master)](https://travis-ci.org/stack-of-tasks/sot-core)
[![Coverage Status](https://coveralls.io/repos/stack-of-tasks/sot-core/badge.png)](https://coveralls.io/r/stack-of-tasks/sot-core)


This software provides a set of dynamic graph plug-ins which can be
used to define and solve hierarchical tasks.


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.
