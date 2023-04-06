# Unity Pathfinding Demo

This repository contains a simple yet effective pathfinding implementation in Unity, demonstrating dynamic pathfinding capabilities with basic optimizations. The demo is designed to handle paths that get blocked during navigation and can be integrated into your Unity projects.

## Features

- Dynamic pathfinding
- Basic optimizations
- Handles blocked paths during navigation

## Getting Started

1. Clone the repository or download the source code.
2. Open the project in Unity.
3. Load the `TestScene` scene from the `Assets` folder.
4. Hit the `Play` button to run the demo.

## Usage

To use the pathfinding functionality in your project, you need to call the `FindPath` method from the `Pathfinding` class. The method takes three arguments: `start`, `finish`, and `callback`.

Here's an example code block:

```csharp
Pathfinding pathfinding = new Pathfinding();
Vector3 start = new Vector3(0, 0, 0);
Vector3 finish = new Vector3(10, 0, 10);

pathfinding.FindPath(start, finish, (path) => {
    if (path != null) {
        // Process the path
    } else {
        // Handle the case when no path is found
    }
});
```

## Future Improvements

- Implement DOTS or Burst Compiler for better performance.
- Enable HPA (hierarchical pathfinding using clusters of smaller grid graphs) for more complex scenarios.
- Profile and optimize the code to address low-level performance issues.

## Architecture

The current architecture does not have a clear separation between Unity and Pathfinding systems due to the simplicity of the project. However, this can be improved upon as the project grows.

## Acknowledgements

This project is inspired by various Unity pathfinding projects available on GitHub.

The A* algorithm implementation is based on the YouTube tutorial by Sebastian Lague: https://www.youtube.com/watch?v=-L-WgKMFuhE&list=PLFt_AvWsXl0cq5Umv3pMC9SPnKjfp9eGW

Sebastian's original tutorial was made into a GitHub repo that can be found at: https://github.com/SebLague/Pathfinding
