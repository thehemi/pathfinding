﻿using UnityEngine;

public class GridRepresentation : Grid
{

    public GameObject nodeMesh;

    public override void DrawGrid(Node node)
    {
        if (node == null)
            return;

        var pos = node.worldPosition;
        //pos.y /= terrainOffset - 0.1f;
        pos.y += 0.1f;
        var nodeInstance = Instantiate(nodeMesh, pos, transform.rotation) as GameObject;
        nodeInstance.transform.localScale = Vector3.one * (nodeDiameter - 0.1f) * 0.1f;
        node.NodeMesh = nodeInstance;
        if (node.NodeMesh.GetComponent<GridColor>())
        {
            node.NodeMesh.GetComponent<GridColor>().UpdateColor(node.walkable);
        }
    }

}
