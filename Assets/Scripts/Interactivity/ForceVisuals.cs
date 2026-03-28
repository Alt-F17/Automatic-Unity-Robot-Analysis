using System.Diagnostics;
using System.Drawing;
using System.Threading.Tasks.Dataflow;
using UnityEngine;

public class ForceVisualizer : MonoBehaviour
{
    void Start()
    {
        robot = GetComponent<RigidBody>();
    }
    void onDrawGizmos(){
    Gizmos.color = Color.green;
    Gizmos.DrawRay(transform.position, transform.forward * 2);
    }
    void Update()
    {
        Debug.DrawRay(Transform.position, appliedForce, Color.red);
    }
}

