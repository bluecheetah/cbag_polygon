#! /usr/bin/env bash

# location of various tools
export BAG_TOOLS_ROOT=/tools/bag3/core

# PATH setup
export PATH=${BAG_TOOLS_ROOT}/bin:${PATH}

# LD_LIBRARY_PATH setup
export LD_LIBRARY_PATH=${BAG_TOOLS_ROOT}/lib64:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${BAG_TOOLS_ROOT}/lib:${LD_LIBRARY_PATH}

# compiler settings
export CMAKE_PREFIX_PATH=${BAG_TOOLS_ROOT}
