using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Unit : MonoBehaviour
{
    #region Public Variables

    [Header("General Settings")] public GameObject Astar;
    public bool drawGizmos = false;
    public Transform target;

    [Header("Movement Settings")] public float movementSpeed = 20;
    public float rotationSpeed = 85;
    public float gravity = 9.8f;
    public float distanceToWaypoint = 1;
    public float stopBeforeDistance = 2;
    public float collisionDetectionDistance = 2.0f;
    public int jumpSpeed = 1;

    [Header("Pathfinding Settings")] public float period = 5f;
    public Vector2 currentPosition = new Vector2(0, 0);
    public int spacesMoved = 0;

    #endregion

    #region Member Variables

    private float m_verticalSpeed = 0;
    protected Vector3[] m_path;
    protected int m_targetIndex;
    private Vector3 lastTargetPosition;
    private CharacterController m_characterController;
    private Node lastNodePosition;
    private List<Node> lastPositionNeighbors;
    private Vector3 m_lastKnownPosition;
    private Quaternion m_lookAtRotation;
    private Grid m_grid;
    private Coroutine lastRoutine = null;
    private bool preventExtraNodeUpdate = false;

    private float nextActionTime;
    private bool isSafeToUpdatePath = false;
    private int pathFoundCount = 0;
    protected bool isMoving = false;
    protected bool isTargetReached = false;

    #endregion

    public virtual void Awake()
    {
        if (Astar != null)
            m_grid = Astar.GetComponent<Grid>();
    }

    public virtual void Start()
    {
        target = transform;
        m_characterController = GetComponent<CharacterController>();
        Pathfinding.Instance.RequestPath(transform.position, target.position, OnPathFound);
        nextActionTime = period;
    }

    public virtual void Update()
    {
        HandleCollisionDetection();
        HandlePathUpdate();
        HandleObstacleJump();
    }

    private void HandleCollisionDetection()
    {
        var right = transform.TransformDirection(Vector3.forward + Vector3.right).normalized *
                    collisionDetectionDistance;
        var left = transform.TransformDirection(Vector3.forward + Vector3.left).normalized * collisionDetectionDistance;

        DetectRaycastCollision(right, transform.position, collisionDetectionDistance);
        DetectRaycastCollision(left, transform.position, collisionDetectionDistance);
    }

    private void HandlePathUpdate()
    {
        if (Time.time > nextActionTime)
        {
            nextActionTime += period;
            isSafeToUpdatePath = true;
        }
        else
        {
            isSafeToUpdatePath = false;
        }

        if (isSafeToUpdatePath || (!isMoving && isTargetReached && !preventExtraNodeUpdate))
        {
            preventExtraNodeUpdate = true;
            UpdateNodePosition();
        }

        if (spacesMoved % 20 == 0 && isSafeToUpdatePath)
        {
            UpdatePath();
        }
        else if (DetectForwardCollision() != null && isSafeToUpdatePath)
        {
            UpdatePath();
        }
        else if (target.position != lastTargetPosition)
        {
            isMoving = true;
            UpdateNodePosition();
            UpdatePath();
        }

        lastTargetPosition = target.position;
    }

    private RaycastHit? DetectForwardCollision()
    {
        var forward = transform.TransformDirection(Vector3.forward) * collisionDetectionDistance;
        var isForwardCollision = DetectRaycastCollision(forward, transform.position, collisionDetectionDistance);

        if (isForwardCollision == null ||
            ((RaycastHit) isForwardCollision).transform.gameObject.GetComponent<Unit>() == null) return null;

        if (!((RaycastHit) isForwardCollision).transform.gameObject.GetComponent<Unit>().isMoving)
        {
            return (RaycastHit) isForwardCollision;
        }

        return null;
    }

    private void HandleObstacleJump()
    {
        var lowerForward = transform.TransformDirection(Vector3.forward) * collisionDetectionDistance;
        var islowerForwardCollision = DetectRaycastCollision(lowerForward,
            (transform.position + new Vector3(0, -0.5f, 0)), collisionDetectionDistance);

        if (islowerForwardCollision != null && m_characterController.isGrounded &&
            ((RaycastHit) islowerForwardCollision).transform.tag == "Jumpable")
        {
            m_verticalSpeed = jumpSpeed;
        }
    }

    public void UpdatePath()
    {
        lastNodePosition.walkable = Walkable.Passable;
        Pathfinding.Instance.RequestPath(transform.position, target.position, OnPathFound);
    }

    public virtual void OnPathFound(Vector3[] newPath, bool pathSuccessful)
    {
        if (pathSuccessful)
        {
            pathFoundCount++;
            m_path = newPath;
            m_targetIndex = 0;

            if (lastRoutine != null)
                StopCoroutine(lastRoutine);

            lastRoutine = StartCoroutine(FollowPath());
        }
    }

    public virtual IEnumerator FollowPath()
    {
        var currentWaypoint = m_path[0];
        while (true)
        {
            if (Vector3.Distance(transform.position, currentWaypoint) < distanceToWaypoint)
            {
                m_targetIndex++;

                if (m_targetIndex >= m_path.Length)
                {
                    isMoving = false;
                    yield break;
                }

                currentWaypoint = m_path[m_targetIndex];
            }

            UpdatePosition(currentWaypoint);
            yield return null;
        }
    }

    public virtual void UpdatePosition(Vector3 destination)
    {
        var node = m_grid.NodeFromWorldPoint(transform.position);

        var direction = destination - transform.position;
        m_verticalSpeed -= Mathf.Clamp(gravity * Time.deltaTime, 0, 30);

        float penalty = node.movementPenalty == 0 ? 1 : node.movementPenalty;
        var movement = new Vector3(0, m_verticalSpeed, 0) +
                       (100 - penalty) * movementSpeed * direction.normalized / 100 * Time.deltaTime;
        m_characterController.Move(movement);
    }

    public virtual void UpdateRotation()
    {
        m_lastKnownPosition = target.transform.position;
        m_lookAtRotation = Quaternion.LookRotation(m_lastKnownPosition - transform.position);
        if (transform.rotation != m_lookAtRotation)
            transform.rotation =
                Quaternion.RotateTowards(transform.rotation, m_lookAtRotation, rotationSpeed * Time.deltaTime);
    }

    public void UpdateNodePosition()
    {
        var node = m_grid.NodeFromWorldPoint(transform.position);

        if (isMoving == false)
        {
            UpdateNeighborsToBlocked(node);
            node.walkable = Walkable.Blocked;
            lastNodePosition = node;
            currentPosition = new Vector2(node.X, node.Y);
            return;
        }

        if (lastNodePosition != null && isMoving)
        {
            preventExtraNodeUpdate = false;
            lastNodePosition.walkable = Walkable.Passable;
            UpdateNeighborsToPassable(node);

            if (!node.Equals(lastNodePosition))
                spacesMoved++;
        }
        else
        {
            node.walkable = Walkable.Blocked;
            lastNodePosition = node;
            currentPosition = new Vector2(node.X, node.Y);
        }
    }

    private void UpdateNeighborsToBlocked(Node node)
    {
        lastPositionNeighbors = m_grid.GetNeighbors(node);
        foreach (var n in lastPositionNeighbors)
        {
            if (n.walkable != Walkable.Impassable)
                n.walkable = Walkable.Blocked;
        }
    }

    private void UpdateNeighborsToPassable(Node node)
    {
        lastPositionNeighbors = m_grid.GetNeighbors(node);
        if (lastPositionNeighbors == null) return;
        foreach (var n in lastPositionNeighbors)
        {
            if (n.walkable != Walkable.Impassable)
                n.walkable = Walkable.Passable;
        }
    }

    public void OnDrawGizmos()
    {
        if (!drawGizmos)
            return;

        if (m_path == null) return;
        for (var i = m_targetIndex; i < m_path.Length; i++)
        {
            Gizmos.color = Color.black;
            Gizmos.DrawCube(m_path[i], Vector3.one);

            Gizmos.DrawLine(i == m_targetIndex ? transform.position : m_path[i - 1], m_path[i]);
        }
    }

    public RaycastHit? DetectRaycastCollision(Vector3 direction, Vector3 position, float distance)
    {
        var ray = new Ray(position, direction);
        if (Physics.Raycast(ray, out var hit, distance))
        {
            Debug.DrawRay(position, direction, Color.red);
            return hit;
        }
        else
        {
            Debug.DrawRay(position, direction, Color.green);
            return null;
        }
    }
}