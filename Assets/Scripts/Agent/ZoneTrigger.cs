using UnityEngine;
using System;

/// <summary>
/// Attach to TargetZoneB at runtime from RobotAgent.Initialize().
/// Fires OnBoxLanded when the expected box Rigidbody enters the trigger volume.
/// Uses a physics trigger rather than per-step distance polling — fires exactly
/// once per landing, regardless of action step timing or time scale.
/// </summary>
[RequireComponent(typeof(BoxCollider))]
public class ZoneTrigger : MonoBehaviour
{
    private Rigidbody expectedBox;
    private Action onBoxLanded;

    public void Initialize(Rigidbody box, Action callback)
    {
        expectedBox = box;
        onBoxLanded = callback;

        // Ensure a trigger collider sized to catch the box from above.
        BoxCollider col = GetComponent<BoxCollider>();
        col.isTrigger = true;
        col.size   = new Vector3(0.1f, 4f, 0.1f); // Microscopic precision (10cm footprint)
        col.center = new Vector3(0f, 2f, 0f);  // centered above zone surface
    }

    private void OnTriggerEnter(Collider other)
    {
        if (expectedBox == null || onBoxLanded == null) return;

        // Only respond to THIS area's own box
        if (other.attachedRigidbody == expectedBox)
        {
            float distance = Vector3.Distance(new Vector3(other.transform.position.x, 0, other.transform.position.z), new Vector3(transform.position.x, 0, transform.position.z));
            if (distance < 0.15f)
            {
                Debug.Log($"<color=green>[ZoneTrigger] Box entered Zone B!</color>");
                onBoxLanded.Invoke();
            }
        }
    }

    private void OnTriggerStay(Collider other)
    {
        if (expectedBox == null || onBoxLanded == null) return;

        // Keep invoking while inside, RobotAgent will ignore if already ending
        if (other.attachedRigidbody == expectedBox)
        {
            // The box's center must be extremely close (0.1m) to the target center
            float distance = Vector3.Distance(new Vector3(other.transform.position.x, 0, other.transform.position.z), new Vector3(transform.position.x, 0, transform.position.z));
            if (distance < 0.15f)
            {
                onBoxLanded.Invoke();
            }
        }
    }
}
