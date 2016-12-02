# littleswarm
Particle Swarm Optimization, written for swarm search and exploration, prioritizing area coverage.

Now this code seems more like an inverted Ant Colony Optimization.

Current features:

--  Able to cover 1000x1000 area with minor clustering, in around 15 minutes.
--  The path checking of each swarm unit is shown as triangular boxes.
--  Travelled paths are stored in an array and displayed as blue dots in real-time.
--  The finished heatmap will be produced as a red-dot plot.

Current issues:

--  pointArray's size increases linearly per iteration.
--  Minor clustering.
--  No visible application yet.

Future possible optimization/improvements:

--  Use grid to replace pointArray, or interact with it to average out certain coordinates.
--  Use longer boxes for checking.
--  Add additional parameter for checkForward() to adjust box length.
--  Use arbitrary 3D-plots to represent contour and soil humidity.
