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
    [SerializeField] private PlacementConstraints placementConstraints;

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
        if (placementConstraints == null)
        {
            placementConstraints = GetComponent<PlacementConstraints>();
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

        if (placementConstraints != null)
        {
            placementConstraints.NotifyValidPlacement();
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

        if (placementConstraints != null)
        {
            newPos = placementConstraints.ConstrainPosition(newPos);
        }

        transform.position = newPos;

        if (placementConstraints != null)
        {
            placementConstraints.NotifyValidPlacement();
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

