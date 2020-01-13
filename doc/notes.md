# Overview

This is a integer polygon manipulation library written in C++17.

Integer coordinate geometry is very important in the field of VLSI circuit design, and knowing
that all vertices are quantized to a grid make various optimizations possible.  Furthermore,
there are performance benefits if one knows that a geometry only contains 45 degree or 90 degree
edges, which are common cases in VLSI circuits.

## Conventions

1. Points in a positive polygon is listed in counter-clockwise order, points in a negative
   polygon is listed in clockwise order.  Therefore, if you create a polygon with clockwise
   winding and add it to a set, you'll subtract a hole instead.

2. `polygon_90` objects must have even number of vertices.  This will normally be the case, unless
   you have some duplicate vertices.  See polygon_90_data implementation for a way to correct this
   if you are using a your custom class.

3. By default, all classes use a horizontal scan line, which scans from low Y coordinates to high
   Y coordinates.  However, the orientation of the scan line can be changed via template parameter.
   The rest of this documentation will assume a horizontal scan line.

## Class Descriptions

### `interval_data`

This class represents a 1-dimensional interval.  An interval is "physical" if it has positive
length.  An interval is "valid" if it has non-negative length.

Calling functions with invalid intervals generally leads to undefined behavior, except for
the following special cases:

1. ORing (i.e. merging) an invalid interval with a valid one will return the valid interval.
   ORing an invalid interval with an invalid interval will return an invalid interval.

2. ANDing (i.e. intersecting) an invalid interval with any interval will return an invalid
   interval.  ANDing two strictly-disjoint (not touching) intervals will result in
   an invalid interval.

### `point_data`

This class represents a 2D point.  In addition, various arithmetic operations are
implemented so this class can also be used as a vector.

### `rectangle_data`

This class represents a rectangle.  A rectangle is "physical" if it has positive width
and height.  A rectangle is "valid" if it has non-negative width and height.

Calling functions with invalid rectangles generally leads to undefined behavior.  Except for
the following special cases:

1. ORing (i.e. merging) an invalid rectangle with a valid one will return the valid
   rectangle. ORing an invalid rectangle with an invalid rectangle will return an invalid
   rectangle.

2. ANDing (i.e. intersecting) an invalid rectangle with any rectangle will return
   an invalid rectangle.  ANDing two strictly disjoint (not touching) rectangles will result
   in an invalid rectangle.

3. transforming an invalid rectangle will return an invalid rectangle.

4. adding an invalid rectangle to any polygon sets is a no-op.

5. convolving a polygon set with a rectangle with negative width or height will shrink that
   dimension.

### Polygon Data Classes

The polygon data classes are just thin wrappers around point vectors whose sole purpose is to
differentiate between 90 degrees, 45 degrees, and general polygons.  As the result, these
classes do not perform any input validation or adjustment, and it is up to the user to ensure
correctness (for example, by calling the `clean()` method).  In particular, equality operator
of polygon classes only checks if the vertex list is identical, so two polygons with rotated
vertex list will be considered different.  Thus, it is advised to call `clean()` on
the polygons before checking for equality.

A polygon can be either positive or negative.  By convention, the points of a positive polygon
is listed in counter-clockwise order, and the points of a negative polygon is in clockwise
order.  When added to a set, a negative polygon will subtract out any overlapping geometry.

To specify a polygon, the first and last vertex can either be the same or not.  It is more
memory efficient to not repeat the starting vertex.

#### `polygon_90_data`

This class represents a polygon with only manhattan edges.  In this special case, you can
reconstruct the polygon just from the X cooordinates of the even vertices and Y coordinates
of the odd vertices, and thus reduce the memory usage by a factor of two.  As the result,
this class will process the given vertex list and store only the necessary coordinates.
Alternatively, you can specify the "compact coordinate list" directly.

because of the vertex processing, the following behavor will occur:

1. if the vertex list has less than 4 elements, the polygon will be set to empty.

2. if the vertex list has odd number of elements (meaning there's a double vertex somewhere),
   an extra duplicate vertex will be added to make the number of elements be even.

3. if the vertex list begins with a vertical edge, the vertices will be shifted by one.
   In other words, the first edge of a `polygon_90_data` is always horizontal.

#### `polygon_45_data`

This class is identical to `polygon_data` except for the traits and class name, and no validation
is done to make sure that the vertices given indeed only have 45 degree edges.  It is up to
the user to ensure correctness.

## `polygon_set_data`

`polygon_set_data` is the generic geometry class that stores one or multiple polygons, and
provides various methods for manipulating the geometry.  It keeps track of whether whether it
only contains 90 degree edges or 45 degree edges, and if so, will select the proper algorithm
to get performance improvement.

`polygon_set_data` stores the vertices of all the added polygons, which is very efficient
for 45 degree or 90 degree case, where you only have finite number of possible edges.  It will
then reconstruct the polygons when needed using scan line algorithm.

### `clean()`

the overall strategy of the "vertex cleaning" algorithm for 45 degree case is described below:

1. first, sort all the vertices, first by Y coordinate, then by X coordinate.  For duplicate vertices,
   sum up all the edges by edge types.  in edges have a value of 1, and out edges have a value of -1.
   we will then process these vertices in other.

2. We keep a sorted map (by X coordinate) of all the upward edges intersecting/ending at the scan line.
   Associated with each edge is the polygon count of the region to the right of that edge.  Then, when
   we process a new vertex, we check every edge connected to it, and see if it turns the polygon count of
   a region from 1 to 0 or from 0 to 1.  If it did, then this must be an actual edge, and we need to add it
   to the final result.  Finally, we update the edges at this X coordinate by replacing the old upward edges
   by the new upward edges.

3. At the end, we will have a "clean" polygon set, where it only contains vertices and edges at the boundaries
   of the geometry.

However, there are some details that one needs to be careful about:

1. in the case where a vertex has a horizontal right edge, the polygon count of the upper right and lower
   right region will be different.  To account for this, we keep track of the difference of the polygon count
   in the last upper and lower region we visit.  Therefore, if this difference is non-zero, we know we're walking
   along a horizontal edge.

2. We need to process intersections in addition to vertices.  To do so, whenever a new upward edge is added,
   we find the intersection point of that edge with the two immediate neighboring edges, and add the intersection
   points to a queue.  We process the union of intersections and vertices in sorted order.  Intersection between
   horizontal edges and other edges are detected and processed in the same way.  Intersections are treated as
   vertices with no up or down edges, so that processing them will not remove edges that pass through them.

3. when we see an intersection point, we need to check if edges actually intersect there (as earlier vertices
   could've terminated one of the edge).

4. If we have an off-grid intersection, we need to check if it is actually on the boundary.  If it is,
   we raise an error, but otherwise, we need to swap the order of the two intersecting edges in the sorted
   map (as they crossed over each other).

5. the generic strategy outlined before will work for all vertices except for a vertical/diagonal collinear
   vertex.  It's upward and downward edges both changes positive-ness of the polygon count, but they
   actually line up so the vertex should be ignored.  We need to add an extra check for that.

### Boolean Operations

binary boolean operations are actually exactly like the cleaning algorithm, but instead of using an integer
polygon count, we need a 2-element tuple polygon count instead, with an element for each operand, and
"positive-ness" is a function of both counts (that depends on the exact boolean operation).  Therefore, we
simply specify the count class as a template parameter to reuse the same code.

### Polygon construction

To recover the polygons from the vertices, we first clean the `polygon_set_data` to remove redundant vertices
and edges.  Note that after cleaning, the following properties holds true:

1. each vertex has even number of edges.

2. edges attached to a vertex alternate in direction.

We construct the polygons from bottom up (following the scan line).  At each Y position, we keep
track of the top-left and top-right edges of each polygon.  At each vertex, we do the following:

1. if it has two or more down edges that belongs to the same polygon, we "complete" this polygon (see later).

2. if the last vertex belongs to a "right open" polygon, and this current vertex has either an up edge or
   a down edge, we add this edge to that "right open" polygon and close it.

3. if we have 2 or more down edges, pick 2 of those edges and join the corresponding polygons together.

4. if we still have a down edge, check how many up edges we have.  If we have even up edges, then we must have
   a horizontal right edge, add the horizontal right edge to the down edge polygon, and mark it as a "right open"
   polygon.  Otherwise, join the down edge with a up edge.

5. if we have 2 or more up edges, pick two of them and create a new polygon with the current vertex as the
   "pointy bottom" vertex.

6. if we still have a up edge, create a new "right open" polygon by joing that up edge with the right horizontal
   edge.

to "complete" a polygon, we check it's positive-ness.  If it's a positive polygon, just output it.  Otherwise,
if it's a negative polygon (meaning it's a hole), we find the polygon immediately to the left, then fracture
this hole by appending it to that polygon.

### Convolve Rectangle

To convolve a polygon set with a rectangle, we first reconstruct individual polygons.  For each of those
polygons, we then overlay the rectangle on each vertex, then move the rectangle along the boundary, and
record the boundary of the new polygon.  These new polygons then replaces the original polygons.  For this
approach to work, we need to be able to figure out at every vertex which corner of the rectangle is on the
boundary of the new polygon.  By drawing some pictures you can figure out how to find this corner, however,
the tricky part comes from concave vertices of the polygon, for which the inward edge and the outward edge
has different rectangle corners.  To resolve this issue, whenever the corners corresponding to the inward and
outward edge differ, we walk along the current rectangle in counter-clockwise direction to join the two
different corners together.  This will result in self-intersecting polygons, but they can be cleaned up by
`polygon_set_data`.

### 90 Degree Algorithms

The algorithm for 90 degree polygons is very similar (as they are just a special case of 45 degree polygons),
but the following optimizations are done:

1. vertical edges never intersect each other, so we don't have to worry about intersections.

2. vertex can only have at most 1 up edge and at most 1 down edge, so polygon construction algorithm can be
   simplified heavily.

3. since we don't have diagonals, X coordinates will never change as we move the scan line up.

4. By using compact coordinate representation to perform the rectangle convolution, it will magically solve
   the concave corner intersections, so you don't want to perform the rectangle walking.
