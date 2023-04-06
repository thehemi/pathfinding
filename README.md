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

This *IS NOT* production looking code, because I'd have to bring in a lot of boilerplate and overkill. Please consider this me adapting the solution to the task. However, I'll run through what I would have done differently to show my awareness of the differences:
 * In production code we'd have very clear patterns for where data is defined, for example, perhaps we avoid the editor and instantiate all components via code (AddComponent etc), or perhaps everything is through prefabs or you can implement annotation classes that do cool funky stuff like load prefabs from disk into GameObjects using [Load("prefab.prefab"). You never want to see a bunch of Start() code that is loading. Encapsulate and abstract and avoid repetition!
 * The question of whether there should be a divide between Unity dependent code and Unity code arises. For example, in an MMO dedicated server you probably want to abandon Unity altogether, so shared components would not have a dependency.
 * If I was implementing pathfinding for a real project, I'd create it as a library. I like to follow a customer/developer model even for internal features. It increases reusability, and forces good design patterns even if reuse will not occur
 * I would also add performance profiling, ideally writing to a network API so I can track usage and make improvements
 * I would probably use ECS or at least Burst compiler

## Acknowledgements

This project is inspired by various Unity pathfinding projects available on GitHub.

The A* algorithm implementation is based on the YouTube tutorial by Sebastian Lague: https://www.youtube.com/watch?v=-L-WgKMFuhE&list=PLFt_AvWsXl0cq5Umv3pMC9SPnKjfp9eGW

Sebastian's original tutorial was made into a GitHub repo that can be found at: https://github.com/SebLague/Pathfinding
