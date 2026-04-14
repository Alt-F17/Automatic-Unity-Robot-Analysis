// be sure to install the sentis package from the package manager to use this script
using UnityEngine;
using Unity.Sentis;

public class ModelLoader : MonoBehaviour
{
    private const int ObservationSize = 26;

    [Header("Model")]
    [SerializeField] private ModelAsset onnxModel;

    [Header("Environment References")]
    [SerializeField] private RobotAgentDupe robotAgent;

    [Header("Inference")]
    [SerializeField] private BackendType backend = BackendType.GPUCompute;
    [SerializeField, Min(1)] private int fixedUpdatesPerInference = 1;
    [SerializeField] private bool verboseLogs;

    private readonly float[] observationBuffer = new float[ObservationSize];
    private static readonly TensorShape InputShape = new TensorShape(1, ObservationSize);

    private Model runtimeModel;
    private IWorker worker;
    private int fixedUpdateCounter;

    public Model RuntimeModel => runtimeModel;

    private void Start()
    {
        if (onnxModel == null)
        {
            Debug.LogError("ModelLoader: ONNX model asset is not assigned.");
            enabled = false;
            return;
        }

        if (robotAgent == null)
        {
            Debug.LogError("ModelLoader: RobotAgentDupe reference is not assigned.");
            enabled = false;
            return;
        }

        runtimeModel = Unity.Sentis.ModelLoader.Load(onnxModel);
        worker = WorkerFactory.CreateWorker(BackendType, runtimeModel, verboseLogs);

        if (verboseLogs)
        {
            Debug.Log($"ModelLoader: model loaded with backend {backend}.");
        }
    }

    private void FixedUpdate()
    {
        if (worker == null || robotAgent == null)
        {
            return;
        }

        fixedUpdateCounter++;
        if (fixedUpdateCounter < fixedUpdatesPerInference)
        {
            return;
        }

        fixedUpdateCounter = 0;
        RunModel();
    }

    private void RunModel()
    {
        if (!robotAgent.TryFillObservationBuffer(observationBuffer))
        {
            return;
        }

        using var inputTensor = new TensorFloat(InputShape, observationBuffer);
        worker.Execute(inputTensor);

        using var outputTensor = worker.PeekOutput() as TensorFloat;
        if (outputTensor == null)
        {
            Debug.LogError("ModelLoader: model output tensor was null.");
            return;
        }

        outputTensor.MakeReadable();

        float baseControl = Mathf.Clamp(outputTensor[0], -1f, 1f);
        float shoulderControl = Mathf.Clamp(outputTensor[1], -1f, 1f);
        float elbowControl = Mathf.Clamp(outputTensor[2], -1f, 1f);

        robotAgent.ApplyActionsInference(baseControl, shoulderControl, elbowControl);
    }

    private void OnDestroy()
    {
        worker?.Dispose();
        worker = null;
    }
}