using UnityEngine;
using UnityEngine.EventSystems;
using TMPro;

[RequireComponent(typeof(BoxCollider))]

// Creates the box drag interactivity for the user 

public class BoxDrag : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler
{
    private Vector3 offset;
    private Camera mainCamera;
    private CanvasGroup canvasGroup;
    private float mZcoord;
    private Vector3 worldOffset;
    private Vector3 boxStartPosition;
    private Vector3 targetInitialPosition;

    public Transform targetZoneA;
    public Transform targetZoneB;
    public Agent robotAgent;

    public Rigidbody movableBox;
    public InferenceController inference;

    [SerializeField] private Canvas canvas;

    ModelLoader myModelLoader = new ModelLoader();

    private void Awake()
    {
        canvasGroup = GetComponent<CanvasGroup>();
        // position of square where the box starts
        boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
        targetInitialPosition = targetZoneB.position + Vector3.up * 0.5f;
        robotAgent.enabled = false;
        inference.enabled = false;
    }

    public void OnBeginDrag(PointerEventData eventData)
    {
        mainCamera = Camera.main;
        canvasGroup.alpha = 0.6f;
        canvasGroup.blocksRaycasts = false;

        mZcoord = mainCamera.WorldToScreenPoint(targetZoneB.transform.position).z;
        worldOffset = targetZoneB.transform.position - GetMouseWorldPos();
    }

    private Vector3 GetMouseWorldPos()
    {
        Vector3 mousePoint = Input.mousePosition;
        mousePoint.z = mZcoord;
        return mainCamera.ScreenToWorldPoint(mousePoint);
    }

    public void OnEndDrag(PointerEventData eventData)
    {
        canvasGroup.alpha = 1f;
        canvasGroup.blocksRaycasts = true;

        // reset the episode when the box is dropped, whether in the target zone or not, to allow for a new attempt
        robotAgent.enabled = true;
        inference.enabled = true;
        
        robotAgent.SetTargetPosition(targetZoneB.position + Vector3.up * 0.5f);
        robotAgent.EndEpisode();
    }

    public void OnDrag(PointerEventData eventData)
    {
        Vector3 newPos = GetMouseWorldPos() + worldOffset;
        transform.position = newPos;
    }

    public void ResetBoxPosition()
    {
        movableBox.position = boxStartPosition;
        myModelLoader.Destroy();
    }

    // public void OnDrop(PointerEventData eventData)
    // {
    //     Throw new System.NotImplementedException();
    // }

}

// Setting the limits that the user can drag the target zone for the box

public class DragLimit : MonoBehaviour
{
    public float xMin, xMax,zMin, zMax;
    public TextMeshProUGUI messageText;

    private void Update()
    {
        Vector3 pos = transform.position;
        pos.x = Mathf.Clamp(pos.x, xMin, xMax);
        pos.z = Mathf.Clamp(pos.z, zMin, zMax);
        transform.position = pos;
    }

    private void AllowedDistance()
    {
        distance = Vector3.Distance(boxStart.transform.position, boxEnd.transform.position);
        if(distance < 50)
        {
            messageText.text = "End position of box is too close";
            targetZoneB.position = targetInitialPosition;
            ResetBoxPosition();
        }
    }
}