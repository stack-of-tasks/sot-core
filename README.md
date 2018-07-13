sot-core
========

[![Building Status](https://travis-ci.org/stack-of-tasks/sot-core.svg?branch=master)](https://travis-ci.org/stack-of-tasks/sot-core)
[![Pipeline status](https://gepgitlab.laas.fr/stack-of-tasks/sot-core/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/stack-of-tasks/sot-core/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/sot-core/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/sot-core/master/coverage/)


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
