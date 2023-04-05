using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


/// <summary>
/// Thoughts on doing this better:
/// -Depends on the actual use-cases. Hierarchical pathfinding (HPA) is a good idea if you have a lot of units that need to move around a lot.
/// -If only moving short distances, could have the grid follow the player and only update the grid when the player moves a certain distance.
/// -Could use a priority queue for the open set instead of a heap. This would be faster for large grids, but slower for small grids.
/// -You can hack decent behavior out of simple ray checks
/// -I suspect getting the profiler out will show anything involving the heap is a bottleneck
///
/// Other Improvements/TODO:
/// -Decouple from Unity? I went back and forth on this, because for this demo you really aren't left with much code,
/// but reducing dependencies is nice if this were any more complicated
/// </summary>
public class Pathfinding : MonoBehaviour
{
    /// <summary>
    /// Keeping a queue of paths requested and callbacks
    /// </summary>
    private readonly Queue<PathRequest> pathRequestQueue = new Queue<PathRequest>();
    private PathRequest currentPathRequest;

    private bool isProcessingPath;


    /// <summary>
    /// Lazy request a path and wait for callback
    /// </summary>
    /// <param name="pathStart"></param>
    /// <param name="pathEnd"></param>
    /// <param name="callback"></param>
    public void RequestPath(Vector3 pathStart, Vector3 pathEnd, Action<Vector3[], bool> callback)
    {
        var newRequest = new PathRequest(pathStart, pathEnd, callback);
        pathRequestQueue.Enqueue(newRequest);
        TryProcessNext();
    }

    public IEnumerator FindPath(Node startNode, Node targetNode)
    {
        var waypoints = Array.Empty<Vector3>();
        var pathSuccess = false;
        // If the starting node is not walkable, try to move to an adjacent walkable node
        if (startNode.walkable != Walkable.Passable)
        {
            var neighbors = grid.GetNeighbors(startNode);
            foreach (var n in neighbors)
            {
                if (n.walkable != Walkable.Passable) continue;
                startNode = n;
                break;
            }
        }

        // Proceed with pathfinding only if both the start and target nodes are walkable
        if (startNode.walkable == Walkable.Passable && targetNode.walkable == Walkable.Passable)
        {
            // Initialize the open set (nodes to be explored) as a heap for efficiency, and the closed set (explored nodes) as a hash set
            var openSet = new Heap<Node>(grid.MaxSize);
            var closedSet = new HashSet<Node>();

            // Add the starting node to the open set
            openSet.Add(startNode);

            // Continue exploring nodes while there are nodes in the open set
            while (openSet.Count > 0)
            {
                // Get the node with the lowest total cost (sum of movement cost and heuristic cost) from the open set and add it to the closed set
                var currentNode = openSet.RemoveFirst();
                closedSet.Add(currentNode);

                // If the current node is the target node, the path has been found
                if (currentNode == targetNode)
                {
                    pathSuccess = true;
                    break;
                }

                // Evaluate the neighbors of the current node
                foreach (var neighbor in grid.GetNeighbors(currentNode))
                {
                    // Ignore non-walkable neighbors or neighbors that have already been explored
                    if (neighbor.walkable != Walkable.Passable || closedSet.Contains(neighbor))
                    {
                        continue;
                    }

                    // Calculate the new movement cost to the neighbor, considering the current node's movement cost, the distance to the neighbor, and the neighbor's movement penalty
                    var newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor) + neighbor.movementPenalty;

                    // If the new movement cost is lower than the neighbor's current movement cost, or if the open set does not yet contain the neighbor, update its costs and set its parent to the current node
                    if (newMovementCostToNeighbor >= neighbor.gCost && openSet.Contains(neighbor)) continue;
                    neighbor.gCost = newMovementCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    // If the open set does not yet contain the neighbor, add it
                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                }
            }
        }

        // Wait for the next frame before proceeding
        yield return null;

        // If the path was successfully found, retrace it to obtain the waypoints and notify the request manager
        if (pathSuccess)
        {
            waypoints = SmoothPath(GetPath(startNode, targetNode));
        }
        FinishedProcessingPath(waypoints, pathSuccess);
    }

    /// <summary>
    /// Find forward path
    /// </summary>
    /// <param name="startNode"></param>
    /// <param name="endNode"></param>
    /// <returns> Walkable path</returns>
    public List<Node> GetPath(Node startNode, Node endNode)
    {
        var path = new List<Node>();
        var currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }

        return path;
    }


    /// <summary>
    /// Determine if a path is currently being processed. If not, start the next path.
    /// </summary>
    void TryProcessNext()
    {
        if (isProcessingPath || pathRequestQueue.Count <= 0) return;
        currentPathRequest = pathRequestQueue.Dequeue();
        isProcessingPath = true;
        StartFindPath(currentPathRequest.pathStart, currentPathRequest.pathEnd);
    }

    /// <summary>
    /// Path has been calculated.
    /// </summary>
    /// <param name="path"></param>
    /// <param name="success"></param>
    private void FinishedProcessingPath(Vector3[] path, bool success)
    {
        currentPathRequest.callback(path, success);
        isProcessingPath = false;
        TryProcessNext();
    }

    /// <summary>
    /// 
    /// </summary>
    private struct PathRequest
    {
        public readonly Vector3 pathStart;
        public readonly Vector3 pathEnd;
        public readonly Action<Vector3[], bool> callback;

        public PathRequest(Vector3 _start, Vector3 _end, Action<Vector3[], bool> _callback)
        {
            pathStart = _start;
            pathEnd = _end;
            callback = _callback;
        }

    }

    Grid grid;
    /// <summary>
    /// Generally not a fan of singletons, but this is a good use case for it.
    /// </summary>
    public static Pathfinding Instance;

    void Awake()
    {
        Instance = this;
        grid = GetComponent<Grid>();
    }


    private void StartFindPath(Vector3 startPos, Vector3 targetPos)
    {
        StartCoroutine(FindPath(startPos, targetPos));
    }


    // <summary>
    // Finds the shortest path between two points using the A* algorithm.
    //
    // The complexity and performance of this function depend on the implementation of the A* algorithm,
    // the grid size, and the Heap data structure used for the open set. In general, the time complexity
    // of the A* algorithm is O(b^d), where b is the branching factor (number of neighbors for each node),
    // and d is the depth of the shortest path. The space complexity is also O(b^d) since it stores nodes
    // in both open and closed sets
    // The performance of this function can be improved in several ways, but it may not be dramatically
    // improved without changing the core algorithm or making significant assumptions about the problem:
    //2. Improve the heuristic function used to estimate the cost from the current node
    //to the target node.Using a more accurate heuristic function can significantly reduce
    //the number of nodes explored by the algorithm, leading to faster execution.
    //4. If the environment allows for it, consider using a different pathfinding algorithm
    //like Dijkstra's algorithm or the Jump Point Search algorithm. These algorithms may offer
    //better performance in specific situations.
    ///// </summary>
    /// <param name="startPos">The starting position of the pathfinding request.</param>
    /// <param name="targetPos">The target position of the pathfinding request.</param>
    /// <returns>An IEnumerator that can be used in a coroutine.</returns>
    private IEnumerator FindPath(Vector3 startPos, Vector3 targetPos)
    {
        // Initialize an empty array of waypoints and a flag to indicate whether the pathfinding was successful


        // Convert the start and target positions to nodes on the grid
        var startNode = grid.NodeFromWorldPoint(startPos);
        var targetNode = grid.NodeFromWorldPoint(targetPos);
        return FindPath(startNode, targetNode);
    }

    public int CalculatePathCost(Node currentNode, Node neighbor)
    {
        // Calculate the new movement cost to the neighbor, considering the current node's movement cost, the distance to the neighbor, and the neighbor's movement penalty
        int newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor) + neighbor.movementPenalty;

        return newMovementCostToNeighbor;
    }


    /// <summary>
    /// Nodes to Vector3
    /// </summary>
    /// <param name="path"></param>
    /// <returns></returns>
    Vector3[] SmoothPath(List<Node> path)
    {

        // Reduce path complexity
        var w = SimplifyPath(path);
        var sw = SmoothCurve(w, 1.0f);
        Array.Reverse(sw);
        return sw;

    }


    /// <summary>
    /// Reduce waypoint array. Remove nodes that are in the same direction as the previous node
    /// 
    /// </summary>
    /// <param name="path"></param>
    /// <returns></returns>
    Vector3[] SimplifyPath(List<Node> path)
    {
        var wp = new List<Vector3>();
        var directionOld = Vector2.zero;

        for (var i = 1; i < path.Count; i++)
        {
            // Direction between two nodes
            var directionNew = new Vector2(path[i - 1].X - path[i].X, path[i - 1].Y - path[i].Y);

            // If path has changed direction
            if (directionNew != directionOld)
            {
                // Does not adapt to height of character
                // Maybe put empty game object at ground level
                wp.Add(path[i].worldPosition + Vector3.up);
            }
            directionOld = directionNew;
        }
        return wp.ToArray();
    }


    /// <summary>
    /// Smooth path using Bezier Spline algorithm.
    /// Reference: http://answers.unity3d.com/questions/392606/line-drawing-how-can-i-interpolate-between-points.html
    /// Reference: http://ibiblio.org/e-notes/Splines/Bezier.htm
    /// Reference: http://catlikecoding.com/unity/tutorials/curves-and-splines/
    /// </summary>
    /// <returns> Interpolated path </returns>
    private Vector3[] SmoothCurve(Vector3[] waypoints, float smoothness)
    {
        if (waypoints.Length <= 1)
            return waypoints;

        smoothness = Mathf.Max(smoothness, 1.0f);
        var wpLen = waypoints.Length;
        var curvedLength = (wpLen * Mathf.RoundToInt(smoothness)) - 1;
        var wpSmooth = new List<Vector3>(curvedLength);

        for (var p = 0; p <= curvedLength; p++)
        {
            var t = Mathf.InverseLerp(0, curvedLength, p);
            var points = new List<Vector3>(waypoints);

            for (var j = wpLen - 1; j > 0; j--)
            {
                for (var i = 0; i < j; i++)
                {
                    points[i] = Vector3.Lerp(points[i], points[i + 1], t);
                }
            }

            wpSmooth.Add(points[0]);
        }

        return wpSmooth.ToArray();
    }

    /// <summary>
    /// Get distance from Node A to Node B
    ///
    /// This method of calculating distance is based on the assumption that moving diagonally
    /// on the grid costs 14 units, and moving horizontally or vertically costs 10 units.
    /// This is a common approach used in pathfinding algorithms, such as the A* algorithm,
    /// to estimate the cost of moving between two points on a grid.
    /// </summary>
    /// <param name="nodeA"></param>
    /// <param name="nodeB"></param>
    /// <returns></returns>
    int GetDistance(Node nodeA, Node nodeB)
    {
        var dstX = Mathf.Abs(nodeA.X - nodeB.X);
        var dstY = Mathf.Abs(nodeA.Y - nodeB.Y);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX - dstY);
        return 14 * dstX + 10 * (dstY - dstX);
    }
}
