using UnityEngine;
using UnityEngine.EventSystems;

[RequireComponent(typeof(BoxCollider))]

public class BoxDrag : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler
{
    private Vector3 offset;
    private Camera mainCamera;
    [SerializeField] private Canvas canvas;

    private rectTransform rectTransform;

    private void Awake()
    {
        rectTransform = GetComponent<RectTransform>();
        canvasGroup = GetComponent<CanvasGroup>();
    }

    public void OnBeginDrag(PointerEventData eventData)
    {
        mainCamera = Camera.main;
        canvasGroup.alpha = 0.6f;
        canvasGroup.rayBlocksRaycasts = false;

        mZcoord = mainCamera.WorldToScreenPoint(gameObject.transform.position).z;
        mZOffset = gameObject.transform.position - mainCamera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, mZcoord));
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

    public void OnDrop(PointerEventData eventData)
    {
        Throw new System.NotImplementedException();
    }

}


public class DragLimit : MonoBehaviour
{
    public float xMin, xMax,zMin, zMax;

    private void Update()
    {
        Vector3 pos = transform.position;
        pos.x = Mathf.Clamp(pos.x, xMin, xMax);
        pos.z = Mathf.Clamp(pos.z, zMin, zMax);
        transform.position = pos;
    }
}