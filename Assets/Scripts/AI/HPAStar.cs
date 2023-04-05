
// Here's a basic example of a Hierarchical Pathfinding A* (HPA*) implementation 
// I tried to make this as simple as possible, so it's not optimized for performance.
// And it builds on A* Pathfinding so it's an easy drop-in for additional functionality

using System;
using System.Collections.Generic;

public class HPAStar
{


    public class Edge
    {
        public Node From;
        public Node To;
        public float Cost;

        public Edge(Node from, Node to, float cost)
        {
            From = from;
            To = to;
            Cost = cost;
        }
    }

    public class Cluster
    {
        public int X;
        public int Y;
        public int Width;
        public int Height;

        public Cluster(int x, int y, int width, int height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }
    }

    public List<Node> Nodes;
    public List<Edge> Edges;
    public List<Cluster> Clusters;
    public int ClusterSize;

    public HPAStar(int width, int height, int clusterSize)
    {
        Nodes = new List<Node>();
        Edges = new List<Edge>();
        Clusters = new List<Cluster>();
        ClusterSize = clusterSize;

        // Create clusters
        for (int y = 0; y < height; y += clusterSize)
        {
            for (int x = 0; x < width; x += clusterSize)
            {
                Clusters.Add(new Cluster(x, y, clusterSize, clusterSize));
            }
        }

        // Create nodes and edges
        foreach (var cluster in Clusters)
        {
            CreateNodesAndEdges(cluster);
        }
    }

    public List<Node> FindPath(Node start, Node goal)
    {
        // Find the clusters containing the start and goal nodes
        Cluster startCluster = FindCluster(start);
        Cluster goalCluster = FindCluster(goal);

        if (startCluster == goalCluster)
        {
            // If both nodes are in the same cluster, use the existing A* implementation
            Pathfinding.Instance.FindPath(start, goal);
            return Pathfinding.Instance.GetPath(start, goal);
        }
        else
        {
            // If nodes are in different clusters, use the HPA* graph
            ; Pathfinding.Instance.FindPath(start, goal);//, Nodes, Edges);
            var path = Pathfinding.Instance.GetPath(start, goal);

            // Refine the path by finding paths within individual clusters
            List<Node> refinedPath = new List<Node>();
            for (int i = 0; i < path.Count - 1; i++)
            {
                Node from = path[i];
                Node to = path[i + 1];

                // Use the existing A* implementation to find the path between nodes within the same cluster
                Pathfinding.Instance.FindPath(from, to);
                var subPath = Pathfinding.Instance.GetPath(to, from);

                // Append the sub-path to the refined path, excluding the last node to avoid duplicates
                refinedPath.AddRange(subPath.GetRange(0, subPath.Count - 1));
            }

            refinedPath.Add(goal);
            return refinedPath;
        }
    }

    private Cluster FindCluster(Node node)
    {
        foreach (var cluster in Clusters)
        {
            if (node.X >= cluster.X && node.X < cluster.X + cluster.Width &&
                node.Y >= cluster.Y && node.Y < cluster.Y + cluster.Height)
            {
                return cluster;
            }
        }

        return null;
    }



#pragma warning disable CS0649 // Field 'HPAStar.grid' is never assigned to, and will always have its default value null
    private Grid grid;
#pragma warning restore CS0649 // Field 'HPAStar.grid' is never assigned to, and will always have its default value null



    private void CreateNodesAndEdges(Cluster cluster)
    {
        for (int y = cluster.Y; y < cluster.Y + cluster.Height; y++)
        {
            for (int x = cluster.X; x < cluster.X + cluster.Width; x++)
            {
                // Check for entrances/exits on the left edge
                if (x == cluster.X && x > 0 && grid.IsWalkable(x - 1, y) && grid.IsWalkable(x, y))
                {
                    Node entranceNode = new Node(x, y);
                    Nodes.Add(entranceNode);
                    ConnectNodeToNeighbors(entranceNode);
                }

                // Check for entrances/exits on the right edge
                if (x == cluster.X + cluster.Width - 1 && x < grid.Width - 1 && grid.IsWalkable(x + 1, y) && grid.IsWalkable(x, y))
                {
                    Node entranceNode = new Node(x, y);
                    Nodes.Add(entranceNode);
                    ConnectNodeToNeighbors(entranceNode);
                }

                // Check for entrances/exits on the top edge
                if (y == cluster.Y && y > 0 && grid.IsWalkable(x, y - 1) && grid.IsWalkable(x, y))
                {
                    Node entranceNode = new Node(x, y);
                    Nodes.Add(entranceNode);
                    ConnectNodeToNeighbors(entranceNode);
                }

                // Check for entrances/exits on the bottom edge
                if (y == cluster.Y + cluster.Height - 1 && y < grid.Height - 1 && grid.IsWalkable(x, y + 1) && grid.IsWalkable(x, y))
                {
                    Node entranceNode = new Node(x, y);
                    Nodes.Add(entranceNode);
                    ConnectNodeToNeighbors(entranceNode);
                }
            }
        }
    }

    private void ConnectNodeToNeighbors(Node node)
    {


        // Iterate through all nodes in the graph
        foreach (var otherNode in Nodes)
        {
            if (node != otherNode)
            {
                // Check if the nodes are in adjacent clusters
                Cluster nodeCluster = FindCluster(node);
                Cluster otherNodeCluster = FindCluster(otherNode);
                bool isAdjacent = Math.Abs(nodeCluster.X - otherNodeCluster.X) <= ClusterSize && Math.Abs(nodeCluster.Y - otherNodeCluster.Y) <= ClusterSize;

                if (isAdjacent)
                {
                    // Find the shortest path between the nodes using the A* algorithm
                    Pathfinding.Instance.FindPath(node, otherNode);
                    var path = Pathfinding.Instance.GetPath(node, otherNode);
                    if (path != null)
                    {
                        // Calculate the cost of the path
                        float cost = Pathfinding.Instance.CalculatePathCost(path[0], path[path.Count - 1]);

                        // Create an edge between the nodes with the calculated cost
                        Edge edge = new Edge(node, otherNode, cost);
                        Edges.Add(edge);

                        // Add the edge to both nodes
                        node.Edges.Add(edge);
                        otherNode.Edges.Add(edge);
                    }
                }
            }
        }
    }
}
