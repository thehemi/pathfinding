using System.Collections.Generic;
using UnityEngine;

public enum Walkable { Blocked, Passable, Impassable };

#pragma warning disable CS0659 // Type overrides Object.Equals(object o) but does not override Object.GetHashCode()
public class Node : IHeapItem<Node>
#pragma warning restore CS0659 // Type overrides Object.Equals(object o) but does not override Object.GetHashCode()
{

    private Walkable m_walkable;
    public Walkable walkable
    {
        get
        {
            return m_walkable;
        }
        set
        {
            m_walkable = value;
            if (NodeMesh != null)
                NodeMesh.GetComponent<GridColor>().UpdateColor(value);
        }

    }
    public Vector3 worldPosition;
    public int X;
    public int Y;
    public float height;
    public int movementPenalty;

    public int gCost;
    public int hCost;
    public Node parent;

    public GameObject NodeMesh;

    public List<HPAStar.Edge> Edges;
    int heapIndex;


    public Node(int x, int y)
    {
        X = x;
        Y = y;
    }

    public Node(Walkable _walkable, Vector3 _worldPos, int x, int y, float _height, int _penalty)
    {
        walkable = _walkable;
        worldPosition = _worldPos;
        X = x;
        Y = y;
        height = _height;
        movementPenalty = _penalty;
    }

    public int fCost
    {
        get
        {
            return gCost + hCost;
        }
    }

    public int HeapIndex
    {
        get
        {
            return heapIndex;
        }
        set
        {
            heapIndex = value;
        }
    }

    public int CompareTo(Node nodeToCompare)
    {
        // if F cost is lower
        var compare = fCost.CompareTo(nodeToCompare.fCost);

        // We and lower H cost if F cost is the same.
        if (compare == 0)
        {
            compare = hCost.CompareTo(nodeToCompare.hCost);
        }
        return -compare;
    }

    public override bool Equals(object obj)
    {
        return worldPosition == ((Node)obj).worldPosition;
    }
}
