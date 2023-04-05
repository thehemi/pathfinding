using System.Collections;
using UnityEngine;

/// <summary>
/// Our simple guy that will follow the path
/// </summary>
public class PathPlayerController : Unit
{
    public float moveSpeed = 5f;

    public GameObject cameraObject;
    public float cameraDistance = 10f;
    public float cameraHeight = 5f;

    private Rigidbody rb;
    private Camera mainCamera;

    /// <summary>
    /// Loops over  waypoints, moving towards each one
    /// </summary>
    /// <returns></returns>
    public override IEnumerator FollowPath()
    {
        var currentWaypoint = m_path is { Length: > 0 } ? m_path[0] : transform.position;

        while (true)
        {

            if (Vector3.Distance(transform.position, currentWaypoint) < distanceToWaypoint)
            {
                m_targetIndex++;

                // If we are done with path.
                if (m_targetIndex >= m_path.Length)
                {
                    isMoving = false;
                    yield break;
                }


                currentWaypoint = m_path[m_targetIndex];
            }

            var forward = transform.TransformDirection(Vector3.forward) * stopBeforeDistance;
            var isForwardCollision = DetectRaycastCollision(forward, transform.position, collisionDetectionDistance);
            // Determine if target space is occupied
            if (isForwardCollision != null && ((RaycastHit)isForwardCollision).transform == target)
            {
                isMoving = false;
                isTargetReached = true;
                m_path = null;
                yield break;
            }
            else
            {
                // Occurs each frame
                isMoving = true;
                isTargetReached = false;
                UpdatePosition(currentWaypoint);

            }

            yield return null;

        } // End While
    }

    public override void Start()
    {
        rb = GetComponent<Rigidbody>();
        mainCamera = cameraObject.GetComponent<Camera>();
        base.Start();
    }

    Vector3 newPosition;

    public override void Update()
    {
        //      MoveCharacter();
        //     RotateCharacter();
        UpdateCameraPosition();
        if (Input.GetMouseButtonDown(0))
        {
            var ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out var hit))
            {
                newPosition = hit.point;
                Pathfinding.Instance.RequestPath(transform.position, newPosition, OnPathFound);


            }
        }
    }




    void RotateCharacter()
    {
        float rotation = Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
        transform.Rotate(0, rotation, 0);
    }

    void UpdateCameraPosition()
    {
        mainCamera.transform.position =
            transform.position - transform.forward * cameraDistance + Vector3.up * cameraHeight;
        mainCamera.transform.LookAt(transform);
    }
}
