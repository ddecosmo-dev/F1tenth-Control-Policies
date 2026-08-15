### RTT vs RRT*

Technically, both RRT and RRT should produce a valid path, but RRT* is capable of finding and optimal path. RRT is good at finding fast paths, but they are not optimal. RRT\* *can* find an optimal path because it has the ability to "re-wire" the tree based on a cost function.

The cost function can be anything (we'll probably use some form of curvature as a cost basis for a future race), but here, I implemented a distance-based cost as hinted at in the skeleton. The distance between each node is calculated, and paths are summed, so the total cost of a new node is equal to the total cost of the potential parent (the distance of the path) plus the distance of the new node to the parent. When we minimize this total path distance globally, an optimal, shortest path emerges.