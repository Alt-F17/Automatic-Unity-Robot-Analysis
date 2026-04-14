using UnityEngine;
using UnityEngine.EventSystems;
using TMPro;

[RequireComponent(typeof(BoxCollider))]
[RequireComponent(typeof(CanvasGroup))]

// Creates the box drag interactivity for the user 

public class BoxDrag : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler
{
    [Header("Scene References")]
    [SerializeField] private Transform targetZoneA;
    [SerializeField] private Transform targetZoneB;
    [SerializeField] private RobotAgentDupe robotAgent;
    [SerializeField] private Rigidbody movableBox;
    [SerializeField] private ModelLoader modelLoader;
    [SerializeField] private DragLimit dragLimit;

    [Header("Flow")]
    [SerializeField] private bool disableControllersUntilPlacement = true;

    private Camera mainCamera;
    private CanvasGroup canvasGroup;
    private Renderer targetZoneBRenderer;
    private float pointerDepth;
    private Vector3 worldOffset;
    private Vector3 boxStartPosition;
    private bool placementValid = false;

    private void Awake()
    {
        if (dragLimit == null)
        {
            dragLimit = GetComponent<DragLimit>();
        }

        canvasGroup = GetComponent<CanvasGroup>();
        targetZoneBRenderer = targetZoneB != null ? targetZoneB.GetComponent<Renderer>() : null;

        if (targetZoneA != null)
        {
            boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
        }

        if (disableControllersUntilPlacement)
        {
            if (robotAgent != null) robotAgent.enabled = false;
            if (modelLoader != null) modelLoader.enabled = false;
        }

        if (dragLimit != null)
        {
            dragLimit.ValidatePlacement();
        }
    }

    public void OnBeginDrag(PointerEventData eventData)
    {
        mainCamera = Camera.main;
        if (mainCamera == null || targetZoneB == null)
        {
            return;
        }

        canvasGroup.alpha = 0.6f;
        canvasGroup.blocksRaycasts = false;

        pointerDepth = mainCamera.WorldToScreenPoint(targetZoneB.position).z;
        worldOffset = targetZoneB.position - GetMouseWorldPos();
    }

    private Vector3 GetMouseWorldPos()
    {
        Vector3 mousePoint = Input.mousePosition;
        mousePoint.z = pointerDepth;
        return mainCamera.ScreenToWorldPoint(mousePoint);
    }

    public void OnEndDrag(PointerEventData eventData)
    {
        canvasGroup.alpha = 1f;
        canvasGroup.blocksRaycasts = true;

        // reset the episode when the box is dropped, whether in the target zone or not, to allow for a new attempt
        if (robotAgent != null) robotAgent.enabled = true;
        if (modelLoader != null) modelLoader.enabled = true;

        if(!placementValid)
        {
            Debug.Log("Invalid placement. Please place the box in the green target zone.");
            return;
        }

        if (robotAgent != null && targetZoneB != null)
        {
            robotAgent.SetTargetPosition(targetZoneB.position + Vector3.up * 0.5f);
            robotAgent.EndEpisode();
        }
    }

    public void OnDrag(PointerEventData eventData)
    {
        if (mainCamera == null)
        {
            return;
        }

        Vector3 newPos = GetMouseWorldPos() + worldOffset;

        if (dragLimit != null)
        {
            newPos = dragLimit.ClampPosition(newPos);
        }

        transform.position = newPos;

        if (dragLimit != null)
        {
            dragLimit.ValidatePlacement();
        }
    }

    public void ResetBoxPosition()
    {
        if (movableBox == null)
        {
            return;
        }

        movableBox.position = boxStartPosition;
        movableBox.velocity = Vector3.zero;
        movableBox.angularVelocity = Vector3.zero;
    }

    public void SetPlacementValid(bool isValid)
    {
        placementValid = isValid;

        if (targetZoneBRenderer != null)
        {
            targetZoneBRenderer.material.color = isValid ? Color.green : Color.red;
        }
    }
}

// Setting the limits that the user can drag the target zone for the box

public class DragLimit : MonoBehaviour
{
    [Header("Bounds")]
    [SerializeField] private float xMin;
    [SerializeField] private float xMax;
    [SerializeField] private float zMin;
    [SerializeField] private float zMax;

    [Header("References")]
    [SerializeField] private Transform targetZoneA;
    [SerializeField] private Transform targetZoneB;
    [SerializeField] private TextMeshProUGUI messageText;
    [SerializeField] private BoxDrag dragController;

    [Header("Distance Rule")]
    [SerializeField] private float minimumDistance = 50f;

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

        ValidatePlacement();
    }

    public Vector3 ClampPosition(Vector3 position)
    {
        position.x = Mathf.Clamp(position.x, xMin, xMax);
        position.z = Mathf.Clamp(position.z, zMin, zMax);
        return position;
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
}