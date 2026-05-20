using UnityEngine;

public class InteractiveZoneController : MonoBehaviour
{
    public Transform robotBase; // Assign to RobotArm_Base
    public Transform otherZone; // Assign the other zone to check angle constraints

    public float minRadius = 1.5f;
    public float maxRadius = 3.5f;
    public float minAngleSeparation = 100f; // degrees

    private bool isDragging = false;
    private float yOffset;
    private Camera mainCamera;

    void Start()
    {
        mainCamera = Camera.main;
        yOffset = transform.position.y;
    }

    void OnMouseDown()
    {
        isDragging = true;
    }

    void OnMouseUp()
    {
        isDragging = false;
    }

    void Update()
    {
        if (isDragging)
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            Plane groundPlane = new Plane(Vector3.up, new Vector3(0, yOffset, 0));
            float distance;
            
            if (groundPlane.Raycast(ray, out distance))
            {
                Vector3 point = ray.GetPoint(distance);
                
                // 1. Constrain to same radius as other zone, or clamped radius if there is no other zone constraint
                float currentOtherRadius = otherZone != null ? Vector3.Distance(new Vector3(otherZone.position.x, 0, otherZone.position.z), new Vector3(robotBase.position.x, 0, robotBase.position.z)) : minRadius;
                
                // For demo simplicity, force both to share the exact same radius
                Vector3 dirFromBase = (point - robotBase.position);
                dirFromBase.y = 0;
                
                // You can optionally allow radius changing by dragging, which updates BOTH zones.
                // Let's implement that: clamp the new point's radius.
                float newRadius = Mathf.Clamp(dirFromBase.magnitude, minRadius, maxRadius);
                dirFromBase = dirFromBase.normalized * newRadius;
                
                Vector3 potentialPosition = robotBase.position + dirFromBase;
                potentialPosition.y = yOffset;

                // 2. Check angle constraint relative to the other zone
                if (otherZone != null)
                {
                    Vector3 otherDir = (otherZone.position - robotBase.position);
                    otherDir.y = 0;
                    
                    float angle = Vector3.Angle(dirFromBase, otherDir);
                    if (angle < minAngleSeparation)
                    {
                        // Too close! We must project the vector along the valid boundaries
                        // (This is a simplified block, usually we just don't allow moving it closer, or we clamp to the boundary 100 deg vector).
                        // Let's just reject movement that breaks the angle constraint for a smoother experience:
                        return; // Ignore this frame's drag
                    }

                    // Force the other zone to adopt our newly dragged radius
                    otherZone.position = robotBase.position + (otherDir.normalized * newRadius) + Vector3.up * otherZone.position.y;
                }

                transform.position = potentialPosition;
            }
        }
    }
}
