using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

public static class GameObjectExtensions
{
    // Extension method to get a component or add it if not found
    public static T GetOrAddComponent<T>(this GameObject gameObject) where T : Component
    {
        // Try to get the component from the game object
        T component = gameObject.GetComponent<T>();

        // If the component is not found, add it to the game object
        if (component == null)
        {
            component = gameObject.AddComponent<T>();
        }

        // Return the component
        return component;
    }
}