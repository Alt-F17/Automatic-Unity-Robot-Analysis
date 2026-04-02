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
    public ModelAsset onnxModel;
    public IModel runtimeModel;
    public RobotAgentDupe robotAgent;
    private Worker worker;


    void Start()
    {
        DontDestroyOnLoad(gameObject);
        if (onnxModel != null)
        {   
            runtimeModel = onnxModel.LoadModel();
            worker =  new Worker(modelManager.getModel(), BackendType.GPUComplete);
            Debug.Log("Model loaded successfully in Awake.");
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
        Tensor input = new Tensor(1, obs.Count, obs.ToArray());

        worker.Execute(input);
        Tensor output = worker.PeekOutput();

        // The output is a 1D tensor with 3 values for base, shoulder, and elbow controls

        float baseControl = output[0];
        float shoulderControl = output[1];
        float elbowControl = output[2];

        ApplyActions(baseControl, shoulderControl, elbowControl);

        input.Dispose();
        output.Dispose();
    }

    List<float> ToList(Vector3 v)
    {
        return new List<float> { v.x, v.y, v.z };
    }
        
    }
    void Destroy()
    {
        worker?.Dispose();
    }
    public IModel GetModel()
    {
        return runtimeModel;
    }
    void applyAction(tensor output)
    {
        float baseControl = output[0];
        float shoulderControl = output[1];
        float elbowControl = output[2];

        ApplyJointTorque(baseRotation, baseControl);
        ApplyJointTorque(shoulderJoint, shoulderControl);
        ApplyJointTorque(elbowJoint, elbowControl);

    }
}