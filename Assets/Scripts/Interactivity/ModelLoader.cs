// be sure to install the sentis package from the package manager to use this script
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Sentis;
using Unity.Sentis.Layers;
using System.Runtime.CompilerServices;
using System.ComponentModel;
using System.Threading.Tasks.Dataflow;

public class MLMManager : MonoBehaviour
{

    [Header("Model")]
    public ModelAsset onnxModel;

    [Header("Environment References")]
    public IModel runtimeModel;
    public RobotAgentDupe robotAgent;
    private Worker worker;

    private const float OBS_SIZE = 26;

    void Start()
    {
        DontDestroyOnLoad(gameObject);
        if (onnxModel != null)
        {   
            runtimeModel = onnxModel.LoadModel();
            worker =  new Worker(runtimeModel, BackendType.GPUComplete);
            Debug.Log("Model loaded successfully in Awake.");
        }
        else
        {
            Debug.LogError("ONNX model asset is not assigned in the inspector.");
            return;
        }
    }

    void FixedUpdate()
    {
        RunModel();
    }

    void RunModel()
    {   
        float obs[] = CollectObservations();

        // Convert to tensor
        using var inputTensor = new TensorFloat(new TensorShape(1, OBS_SIZE), obs);

        worker.Schedule(inputTensor);

        using TensorFloat output = (worker.PeekOutput() as TensorFloat).ReadbackAndClone();

        float baseControl = Mathf.Clamp(output[0, 0], -1f, 1f);
        float shoulderControl = Mathf.Clamp(output[0, 1], -1f, 1f);
        float elbowControl = Mathf.Clamp(output[0, 2], -1f, 1f);

        robot.ApplyJointAction(baseControl, shoulderControl, elbowControl);

    }


    float[] CollectObservations()
    {
        List<float> obs = new List<float>(OBS_SIZE);

        Transform agentTransform = robotAgent.transform;
        Transform magnet = robotAgent.Magnet;
        Rigidbody box = robotAgent.MovableBox;
        Transform floor = robotAgent.Floor;
        Transform targetZoneB = robotAgent.TargetZoneB;

        Vector3 boxPos = movableBox != null ? movableBox.position : boxStartPosition;
        Vector3 boxVel = movableBox != null ? movableBox.GetComponent<Rigidbody>().velocity : Vector3.zero;
        Vector3 targetZoneBPos = targetZoneB != null ? targetZoneB.position : targetInitialPosition;
        float localFloorY = floor != null ? floor.position.y : transform.position.y;

        AddVector3(obs, agentTransform.InverseTransformPoint(magnet.position));
        AddVector3(obs, agentTransform.InverseTransformPoint(boxPos));
        AddVector3(obs, agentTransform.InverseTransformPoint(targetZoneBPos));

        Vector3 boxPosXZ = new Vector3(boxPos.x, 0f, boxPos.z);
        Vector3 targetZoneBPosXZ = new Vector3(targetZoneBPos.x, 0f, targetZoneBPos.z);
        obs.Add(Vector3.Distance(boxPosXZ, targetZoneBPosXZ));

        obs.Add(robotAgent.GetNormalizedAngle(robotAgent.baseJoint));
        obs.Add(robotAgent.GetNormalizedAngle(robotAgent.shoulderJoint));
        obs.Add(robotAgent.GetNormalizedAngle(robotAgent.elbowJoint));
        obs.Add(robotAgent.GetJointVelocity(robotAgent.baseJoint));
        obs.Add(robotAgent.GetJointVelocity(robotAgent.shoulderJoint));
        obs.Add(robotAgent.GetJointVelocity(robotAgent.elbowJoint));

        AddVector3(obs, agentTransform.InverseTransformPoint(boxVel));
        AddVector3(obs, agentTransform.InverseTransformPoint(magnet.GetComponent<Rigidbody>().velocity));
        AddVector3(obs, Vector3.zero); // Placeholder for torque observations, to be replaced with actual torque values

        obs.Add(robotAgent.IsBoxAttached ? 1f : 0f);                              
        obs.Add(distToBox < 0.5f ? 1f : 0f);                                       
        obs.Add(Vector3.Distance(boxPos, targetPos) < 0.5f ? 1f : 0f);           
        obs.Add(0f);                                                               
        obs.Add(boxPos.y - floorY);                                                 
        obs.Add(0f);                                                                
        obs.Add(1f);

        // check if the observation count matches the number of nodes passed

        if (obs.Count != OBS_SIZE)
        {
            Debug.LogError($"Observation count {obs.Count} does not match expected size {OBS_SIZE}.");
        }

        return obs.ToArray();
    }

    static void AddVector3(List<float> list, Vector3 vec)
    {
        list.Add(vec.x);
        list.Add(vec.y);
        list.Add(vec.z);
        
    }

    void Destroy()
    {
        worker?.Dispose();
    }
    public IModel GetModel()
    {
        return runtimeModel;
    }
    
    void applyAction(Tensor output)
    {
        float baseControl = output[0];
        float shoulderControl = output[1];
        float elbowControl = output[2];

        ApplyJointTorqueRaw(baseRotation, baseControl);
        ApplyJointTorqueRaw(shoulderJoint, shoulderControl);
        ApplyJointTorqueRaw(elbowJoint, elbowControl);
    }
}