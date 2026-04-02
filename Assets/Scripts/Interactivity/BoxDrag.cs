using System.Numerics;
using UnityEngine;
using UnityEngine.EventSystems;
using TMPro;

[RequireComponent(typeof(BoxCollider))]

ModelLoader myModelLoader = new ModelLoader();

// Creates the box drag interactivity for the user 

public class BoxDrag : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler
{
    private Vector3 offset;
    private Camera mainCamera;

    public Agent targetZoneA;
    public Agent targetZoneB;

    public Agent movableBox;
    [SerializeField] private Canvas canvas;

    private rectTransform rectTransform;


    private void Awake()
    {
        rectTransform = GetComponent<RectTransform>();
        canvasGroup = GetComponent<CanvasGroup>();
        // position of square where the box starts
        boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
        targetInitialPosition = targetZoneB.position + Vector3.up * 0.5f;
    }

    public void OnBeginDrag(PointerEventData eventData)
    {
        mainCamera = Camera.main;
        canvasGroup.alpha = 0.6f;
        canvasGroup.rayBlocksRaycasts = false;

        mZcoord = mainCamera.WorldToScreenPoint(targetZoneB.transform.position).z;
        mZOffset = targetZoneB.transform.position - mainCamera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, mZcoord));
    }


    public void OnMouseDrag(PointerEventData eventData)
    {
        rectTransform.anchoredPosition += eventData.delta / canvas.scaleFactor;
    }

    public void OnEndDrag(PointerEventData eventData)
    {
        Vector3 mousePos = Input.mousePosition;
        mousePos.z = mainCamera.WorldToScreenPoint(transform.position).z;
        transform.position = mainCamera.ScreenToWorldPoint(mousePos) + offset;
        canvasGroup.alpha = 1f;
        canvasGroup.rayBlocksRaycasts = true;
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        Debug.Log("Pointer Down");
        Vector3 mousePos = Input.mousePosition;
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