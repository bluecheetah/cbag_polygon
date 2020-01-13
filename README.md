# CBAG (C++ BAG AMS Generator) Polygon

This is a integer polygon manipulation library written in C++17.  It is primarily designed
to work as the backend of the [CBAG](https://github.com/bluecheetah/cbag).

Integer coordinate geometry is very important in the field of VLSI circuit design, and knowing
that all vertices are quantized to a grid make various optimizations possible.  Furthermore,
there are performance benefits if one knows that a geometry only contains 45 degree or 90 degree
edges, which are common cases in VLSI circuits.

## Status

This library is in pre-alpha stage.  Everything is subjective to change, we do not guarantee
correctness as the code is not thoroughly tested, though some unit tests exist.  Use at your
own risk.

## Licensing

This library is licensed under the Apache-2.0 license.  However, some source files are licensed
under both Apache-2.0 and BSD-3-Clause license, meaning that the user must comply with the
terms and conditions of both licenses.  See [here](LICENSE.BSD-3-Clause) for full text of the
BSD license, and [here](LICENSE.Apache-2.0) for full text of the Apache license.  See individual
files to check if they fall under Apache-2.0, or both Apache-2.0 and BSD-3-Clause.

## Acknowledgement

The API and naming convention is largely inspired by the Boost Polygon library, but the algorithms
are developed independently from scratch.  Furthermore, I made some design decisions that differs
from the Boost Polygon library, such as allowing invalid rectangles as a special case.
