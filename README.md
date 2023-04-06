# Pathfinding
Fairly simple dynamic pathfinding implementation.
-Has basic optimizations
-Handles paths that get blocked during navigation

## Optimization TODO
* DOTS Implementation. Using DOTS or burst compiler would be much faster in theory, but at this scale it's hard to see and the code was much more complicated
* HPA Implementation (hierarchical pathfinding using clusters of smaller grid graphs). I disabled this because it's just overkill for this demo
* Needs profiling to address low level performance issues

## Architecture
* Not a beautiful divide between Unity and Pathfinding systems, but this is such a small project, again, it would be overkill

## Credits

Various Unity Github Pathfinding projects were used for reference

A* algorithm based off YouTube tutorial by Sebastian Lague
https://www.youtube.com/watch?v=-L-WgKMFuhE&list=PLFt_AvWsXl0cq5Umv3pMC9SPnKjfp9eGW

Sebastian's original tutorial was made into a GitHub repo that can be found at:
https://github.com/SebLague/Pathfinding
