using UnityEngine;

/// <summary>
/// literally the simplest possible flycam controller: arrow keys to move, right mouse to look.
/// like... you can't mess this up. just put it on the main camera and you're good to go.
/// </summary>

public class SimpleCameraController : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 20f;
    [SerializeField] private float lookSpeed = 3f;
    private float yaw;
    private float pitch;

    void Start()
    {
        yaw = transform.eulerAngles.y;
        pitch = transform.eulerAngles.x;
    }

    void Update()
    {
        Vector3 move = Vector3.zero;
        if (Input.GetKey(KeyCode.UpArrow)) move += transform.forward;
        if (Input.GetKey(KeyCode.DownArrow)) move -= transform.forward;
        if (Input.GetKey(KeyCode.LeftArrow)) move -= transform.right;
        if (Input.GetKey(KeyCode.RightArrow)) move += transform.right;
        if (Input.GetKey(KeyCode.Space)) move += Vector3.up;
        if (Input.GetKey(KeyCode.LeftShift)) move -= Vector3.up;
        transform.position += move.normalized * moveSpeed * Time.deltaTime;

        if (Input.GetMouseButton(1))
        {
            yaw += Input.GetAxis("Mouse X") * lookSpeed;
            pitch -= Input.GetAxis("Mouse Y") * lookSpeed;
            pitch = Mathf.Clamp(pitch, -90f, 90f);
            transform.eulerAngles = new Vector3(pitch, yaw, 0f);
        }
    }
}
