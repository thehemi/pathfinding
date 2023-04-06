using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SmoothFollow : MonoBehaviour
{
    public Transform player; // Reference to player entity
    public float smoothTime = 0.3f; // Time for the camera to reach the target position
    public Vector3 offset; // Offset between player and camera

    private Vector3 velocity = Vector3.zero; // Velocity used for SmoothDamp

    void Start()
    {
        if(player == null)
        player = FindAnyObjectByType<PathPlayerController>().transform;
        // Set the initial camera position based on the player and offset
        transform.position = player.position + offset;
    }

    void Update()
    {
        OnUpdate();
    }

    void OnUpdate()
    {
        // Calculate the target position based on player position and offset
        Vector3 targetPosition = player.position + offset;

        // Smoothly move the camera towards the target position
        transform.position = Vector3.SmoothDamp(transform.position, targetPosition, ref velocity, smoothTime);
    }
}