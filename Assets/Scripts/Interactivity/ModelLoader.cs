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
        List<float> obs = new List<float>();

        Vector3 boxPos = movableBox != null ? movableBox.position : boxStartPosition;
        Vector3 boxVel = movableBox != null ? movableBox.GetComponent<Rigidbody>().velocity : Vector3.zero;

        float localFloorY = floor != null ? floor.position.y : transform.position.y;

        float obs[] = CollectObservations();

        // Convert to tensor
        using var inputTensor = new TensorFloat(new TensorShape(1, OBS_SIZE), obs);

        worker.Schedule(inputTensor);

        using TensorFloat output = (worker.PeekOutput() as TensorFloat).ReadbackAndClone();

        worker.Execute(inputTensor);

        // The output is a 1D tensor with 3 values for base, shoulder, and elbow controls

        float baseControl = output[0];
        float shoulderControl = output[1];
        float elbowControl = output[2];

        ApplyActions(baseControl, shoulderControl, elbowControl);

        input.Dispose();
        output.Dispose();
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

        obs.AddRange(ToList(boxPos));
        obs.AddRange(ToList(boxVel));
        obs.Add(localFloorY);
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