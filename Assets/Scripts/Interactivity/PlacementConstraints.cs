// Setting the limits that the user can drag the target zone for the box

using TMPro;
using UnityEngine;

[RequireComponent(typeof(CapsuleCollider))]
public class PlacementConstraints : MonoBehaviour
{
    [Header("Bounds")]
    [SerializeField] private float xMin = -100f;
    [SerializeField] private float xMax = 100f;
    [SerializeField] private float zMin = -100f;
    [SerializeField] private float zMax = 100f;
    [SerializeField] private float radius = 100f;

    [Header("References")]
    [SerializeField] private Transform cylindricalBoundary;
    [SerializeField] private Transform minimumDistanceBoundary;
    [SerializeField] private Transform targetZoneA;
    [SerializeField] private Transform targetZoneB;
    [SerializeField] private TextMeshProUGUI messageText;
    [SerializeField] private BoxDrag dragController;

    [Header("Constraint Role")]
    [SerializeField] private bool draggedZoneIsA;

    [Header("Distance Rule")]
    [SerializeField] private float minimumDistance = 50f;

    [Header("Boundary Colliders")]
    [SerializeField] private CapsuleCollider outerBoundary;
    [SerializeField] private CapsuleCollider innerBoundary;

    private float outerRadius;
    private float innerRadius;
    private Vector3 outerCenter;
    private Vector3 innerCenter;

    private Vector3 lastValidDraggedPosition;
    private bool hasValidPosition;

    private Transform DraggedZone => draggedZoneIsA ? targetZoneA : targetZoneB;
    private Transform OtherZone => draggedZoneIsA ? targetZoneB : targetZoneA;

    private void Awake()
    {
        if (dragController == null)
        {
            dragController = GetComponent<BoxDrag>();
        }

        ResolveBoundaryReferences();
        RefreshBoundaryData();
    }

    private void Start()
    {
        if (DraggedZone != null)
        {
            lastValidDraggedPosition = DraggedZone.position;
            hasValidPosition = true;
        }

        NotifyValidPlacement();
    }

    private void Update()
    {
        Transform other = OtherZone;
        if (innerBoundary != null && other != null)
        {
            Vector3 current = innerBoundary.transform.position;
            innerBoundary.transform.position = new Vector3(other.position.x, current.y, other.position.z);
        }

        RefreshBoundaryData();
    }

    public Vector3 ConstrainPosition(Vector3 desiredPosition)
    {
        desiredPosition.x = Mathf.Clamp(desiredPosition.x, xMin, xMax);
        desiredPosition.z = Mathf.Clamp(desiredPosition.z, zMin, zMax);

        Vector2 desiredXZ = new Vector2(desiredPosition.x, desiredPosition.z);
        Vector2 outerCenterXZ = new Vector2(outerCenter.x, outerCenter.z);
        Vector2 fromOuterCenter = desiredXZ - outerCenterXZ;

        if (outerRadius > 0f && fromOuterCenter.magnitude > outerRadius)
        {
            desiredXZ = outerCenterXZ + fromOuterCenter.normalized * outerRadius;
        }

        Vector2 innerCenterXZ = new Vector2(innerCenter.x, innerCenter.z);
        Vector2 fromInnerCenter = desiredXZ - innerCenterXZ;
        if (innerRadius > 0f && fromInnerCenter.magnitude < innerRadius)
        {
            Vector2 pushDirection = fromInnerCenter.magnitude > 0.0001f ? fromInnerCenter.normalized : Vector2.right;
            desiredXZ = innerCenterXZ + pushDirection * innerRadius;
        }

        return new Vector3(desiredXZ.x, desiredPosition.y, desiredXZ.y);
    }

    public void NotifyValidPlacement()
    {
        Transform dragged = DraggedZone;
        Transform other = OtherZone;
        if (targetZoneA == null || targetZoneB == null || dragged == null || other == null)
        {
            return;
        }

        Vector3 clampedPosition = ConstrainPosition(dragged.position);
        if ((clampedPosition - dragged.position).sqrMagnitude > 0.0001f)
        {
            dragged.position = clampedPosition;
        }

        Vector2 a = new Vector2(targetZoneA.position.x, targetZoneA.position.z);
        Vector2 b = new Vector2(targetZoneB.position.x, targetZoneB.position.z);
        float distance = Vector2.Distance(a, b);

        if (distance < minimumDistance)
        {
            if (messageText != null)
            {
                messageText.text = "Zones are too close";
            }

            if (hasValidPosition)
            {
                dragged.position = lastValidDraggedPosition;
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

        lastValidDraggedPosition = dragged.position;
        hasValidPosition = true;

        if (dragController != null)
        {
            dragController.SetPlacementValid(true);
        }
    }

    private void ResolveBoundaryReferences()
    {
        if (outerBoundary == null && cylindricalBoundary != null)
        {
            outerBoundary = cylindricalBoundary.GetComponent<CapsuleCollider>();
        }

        if (innerBoundary == null && minimumDistanceBoundary != null)
        {
            innerBoundary = minimumDistanceBoundary.GetComponent<CapsuleCollider>();
        }

        if (outerBoundary == null)
        {
            outerBoundary = GetComponent<CapsuleCollider>();
        }
    }

    private void RefreshBoundaryData()
    {
        Transform dragged = DraggedZone;
        Transform other = OtherZone;

        if (outerBoundary != null)
        {
            outerRadius = outerBoundary.radius * GetMaxScale(outerBoundary.transform);
            outerCenter = outerBoundary.transform.position;
        }
        else
        {
            outerRadius = Mathf.Max(radius, 0f);
            outerCenter = dragged != null ? dragged.position : transform.position;
        }

        if (innerBoundary != null)
        {
            innerRadius = innerBoundary.radius * GetMaxScale(innerBoundary.transform);
            innerCenter = innerBoundary.transform.position;
        }
        else
        {
            innerRadius = Mathf.Max(minimumDistance, 0f);
            innerCenter = other != null ? other.position : transform.position;
        }
    }

    private static float GetMaxScale(Transform t)
    {
        Vector3 scale = t.lossyScale;
        return Mathf.Max(Mathf.Abs(scale.x), Mathf.Abs(scale.z));
    }
}