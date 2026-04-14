using UnityEngine;

// draw torque vectors for angular force visualization

public class ForceVisuals : MonoBehaviour
{
    [SerializeField] private RobotAgentDupe robot;
    [SerializeField] private bool drawInReleaseBuilds;
    [SerializeField] private float torqueScale = 0.1f;

    private void Start()
    {   
        if (robot == null)
        {
            robot = GetComponent<RobotAgentDupe>();
        }
    }

    private void Update()
    {
        if (robot == null)
        {
            return;
        }

        if (!drawInReleaseBuilds && !Debug.isDebugBuild)
        {
            return;
        }

        DrawTorque(robot.BaseRotationJoint, Color.red);
        DrawTorque(robot.ShoulderJoint, Color.green);
        DrawTorque(robot.ElbowJoint, Color.blue);
    }

    private void DrawTorque(ArticulationBody joint, Color color)
    {
        if (joint == null) return;

        // Use the agent's torque estimate and draw it along the joint local X axis.
        float torqueMagnitude = robot.GetJointTorque(joint);
        Vector3 torque = joint.transform.right * torqueMagnitude;
        Vector3 jointPosition = joint.transform.position;
        Debug.DrawRay(jointPosition, torque * torqueScale, color);
    }
}

