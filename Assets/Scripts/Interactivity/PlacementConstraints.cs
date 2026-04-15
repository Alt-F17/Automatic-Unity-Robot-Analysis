// Setting the limits that the user can drag the target zone for the box

using UnityEngine;
using System.Runtime.Serialization;
using System.Runtime.Intrinsics.X86;
using System.Numerics;
using System.Threading.Tasks.Dataflow;

public class DragLimit : MonoBehaviour
{
    [Header("Bounds")]
    [SerializeField] private float xMin;
    [SerializeField] private float xMax;
    [SerializeField] private float zMin;
    [SerializeField] private float zMax;

    [SerializeField] private float radius;

    [Header("References")]
    
    [SerializeField] private Transform cylindricalBoundary;
    [SerializeField] private Transform minimumDistanceBoundary;

    [SerializeField] private Transform targetZoneA;
    [SerializeField] private Transform targetZoneB;
    [SerializeField] private TextMeshProUGUI messageText;
    [SerializeField] private BoxDrag dragController;

    [Header("Distance Rule")]
    [SerializeField] private float minimumDistance = 50f;

    [Header("Boundary Colliders")]

    [SerializedField] private CapsuleCollider outerBoundary;
    [SerializedField] private CapsuleCollider innerBoundary;

    // Collider data 
    private float outerRadius;
    private float innerRadius;
    private Vector3 outerCenter;
    private Vector3 innerCenter;

        
    // TODO: Change to radius based COLLIDERS in unity itself, create
    // a collider that encompasses the valid area, only let the area within
    // the collider be draggable, and REMOVE (eventually) the valid/invalid
    // states that would reset the position. This would be more intuitive 
    // and less frustrating for the user

    private Vector3 lastValidTargetPosition;
    private bool hasValidPosition;

    private void Start()
    {

        if (targetZoneB != null)
        {
            lastValidTargetPosition = targetZoneB.position;
            hasValidPosition = true;
        }
        if (outerBoundary != null)
        {
            outerRadius = outerBoundary.radius.GetMaxScale(outerBoundary.transform);
            outerCenter = outerBoundary.transform.position;
        }
        if (innerBoundary != null)
        {
            innerRadius = innerBoundary.radius.GetMaxScale(innerBoundary.transform);
        }
    }

    private void Update()
    {
        if (innerBoundary != null && targetZoneA != null)
        {
            innerBoundary.transform.position = new Vector3(targetZoneA.position.x, innerBoundary.transform.position.y, targetZoneA.position.z);
        }
    }

    public Vector3 ConstrainPosition(Vector3 desiredPosition)
    {
        Vector2 desiredXZ = new Vector2(desiredPosition.x, desiredPosition.y);
        Vector2 outerCenterXZ = new Vector2(outerCenter.x, outerCenter.z);

        Vector2 currentDistanceFromRadius = desiredXZ - outerCenterXZ;

        if (currentDistanceFromRadius > outerRadius)
        {
            desiredXZ = outerCenterXZ + currentDistanceFromRadius.normalized * outerRadius;
        }
    }

    public void ValidatePlacement()
    {
        
        if (targetZoneA == null || targetZoneB == null)
        {
            return;
        }

        Vector3 clampedPosition = ClampPosition(targetZoneB.position);
        if ((clampedPosition - targetZoneB.position).sqrMagnitude > 0.0001f)
        {
            targetZoneB.position = clampedPosition;
        }

        // Added to create boundaries for the cylindrical object

        if (transform.position.x > radius)
        {
            targetZoneB.position = new Vector3(radius, transform.position.y, transform.position.z);
        }


        float distance = Vector3.Distance(targetZoneA.position, targetZoneB.position);
        if (distance < minimumDistance)
        {
            if (messageText != null)
            {
                messageText.text = "End position of box is too close";
            }

            if (hasValidPosition)
            {
                targetZoneB.position = lastValidTargetPosition;
            }

            if (dragController != null)
            {
                dragController.SetPlacementValid(false);
            }

            return;
        }

        if (messageText != null)
        {
            messageText.text = string.Empty;
        }

        lastValidTargetPosition = targetZoneB.position;
        hasValidPosition = true;

        if (dragController != null)
        {
            dragController.SetPlacementValid(true);
        }
    }

    private void GetMaxScale(TransformBlock t)
    {
        return MathF.Max(t.lossyScale.x, t.lossyScale.z);
    }
}