using System.Diagnostics;
using System.Drawing;
using System.Numerics;
using System.Threading.Tasks.Dataflow;
using UnityEngine;

// draw torque vectors for angular force visualization

public class ForceVisualizer : MonoBehaviour
{
    public RobotAgentDupe robot;

    void Start()
    {   
        if(robot == null)
        {
            robot = GetComponent<RobotAgentDupe>();
        }
    }

    void Update(){
        DrawTorque(robot.baseJoint, Color.red);
        DrawTorque(robot.shoulderJoint, Color.green);
        DrawTorque(robot.elbowJoint, Color.blue);
    }

    void DrawTorque(ArticulationBody joint, Color color)
    {
        if(joint == null) return;

        Vector3 torque = joint.torque;
        Vector3 jointPosition = joint.transform.position;
        float scale = 0.1f; // Adjust this scale factor as needed for better visualization
        Debug.DrawRay(jointPosition, torque * scale, color);
    }
}

