using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour
{

    public bool displayGridGizmos;
    public LayerMask unwalkableMask;
    public Vector2 gridWorldSize;
    public float nodeRadius;
    public TerrainType[] walkableRegions;
    public float checkRadiusModifier = 2;
    public float terrainOffset = 3;
    LayerMask walkableMask;
    Dictionary<int, int> walkableRegionsDictionary = new Dictionary<int, int>();

    public Node[,] grid;

    protected float nodeDiameter;
    protected int gridSizeX, gridSizeY;

    public int Width { get { return gridSizeX; } }
    public int Height { get { return gridSizeY; } }


    public bool IsWalkable(int x, int y)
    {

        return grid[x, y].walkable == Walkable.Passable;
    }

    void Awake()
    {
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);


        // Note: Layers are stored in a 32 bit int
        foreach (var region in walkableRegions)
        {
            walkableMask.value |= region.terrainMask.value;
            walkableRegionsDictionary.Add((int)Mathf.Log((float)region.terrainMask.value, 2f), region.terrainPenalty);
        }

        CreateGrid();
    }

    public int MaxSize
    {
        get
        {
            return gridSizeX * gridSizeY;
        }
    }

    /// <summary>
    /// For simplicity nodes are game objects. I 
    /// </summary>
    void CreateGrid()
    {
        grid = new Node[gridSizeX, gridSizeY];
        var worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;

        for (var x = 0; x < gridSizeX; x++)
        {
            for (var y = 0; y < gridSizeY; y++)
            {
                var worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
                var walkable = !(Physics.CheckSphere(worldPoint, nodeRadius * checkRadiusModifier, unwalkableMask));

                var movementPenalty = 0;
                float height = 0;
                // raycast
                if (walkable)
                {
                    var ray = new Ray(worldPoint + Vector3.up * 50, Vector3.down);
                    if (Physics.Raycast(ray, out var hit, 100, walkableMask))
                    {

                        // Determine the movement penalty of the terrain type
                        walkableRegionsDictionary.TryGetValue(hit.collider.gameObject.layer, out movementPenalty);

                        // Get the height of a block
                        worldPoint.y = height = (hit.transform.position.y + hit.collider.bounds.extents.y);
                    }
                }

                var walkableEnum = Walkable.Passable;
                if (!walkable)
                    walkableEnum = Walkable.Impassable;

                //worldPoint.y = Mathf.Clamp(worldPoint.y, 0.1f, Mathf.Infinity);
                grid[x, y] = new Node(walkableEnum, worldPoint, x, y, height, movementPenalty);
                DrawGrid(grid[x, y]);
            }
        }
    }

    /// <summary>
    /// Find all nodes around a given node.
    /// </summary>
    /// <param name="node"> node whose neighbors are to be found</param>
    /// <returns></returns>
    public List<Node> GetNeighbors(Node node)
    {
        var neighbors = new List<Node>();
        int[] offsetX = { -1, 0, 1, -1, 1, -1, 0, 1 };
        int[] offsetY = { -1, -1, -1, 0, 0, 1, 1, 1 };

        for (var i = 0; i < offsetX.Length; i++)
        {
            var checkX = node.X + offsetX[i];
            var checkY = node.Y + offsetY[i];

            if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
            {
                neighbors.Add(grid[checkX, checkY]);
            }
        }

        return neighbors;
    }

    /// <summary>
    /// Convert workspace to grid space cartesian coordinates and return node at that point.
    /// </summary>
    /// <param name="worldPosition"> Actual position of node in 3D space</param>
    /// <returns> Node intersecting worldPosition</returns>
    public Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        var percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
        var percentY = (worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;
        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);

        var x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        var y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
        return grid[x, y];
    }

    public virtual void DrawGrid(Node node) { }

    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));
        if (grid != null && displayGridGizmos)
        {
            foreach (var n in grid)
            {
                Gizmos.color = (n.walkable == Walkable.Passable) ? Color.white : Color.red;
                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
            }
        }
    }

    [System.Serializable]
    public class TerrainType
    {
        public LayerMask terrainMask;
        public int terrainPenalty;
    }

}
