// be sure to install the sentis package from the package manager to use this script
using UnityEngine;
using Unity.Sentis;
using Unity.Sentis.Layers;

public class ModelInference : MonoBehaviour
{
    public NNModel model;
    private Model runtimeModel;
    private IWorker worker;

    void Start()
    {
        runtimeModel = ModelLoader.Load(model);
        worker = WorkerFactory.CreateWorker(BackendType.GPUCompute, runtimeModel);
    }

    void runInference(Tensor input)
    {
        using TensorFloat inputTensor = new TensorFloat(input);
        worker.Execute(inputTensor);
        Tensor outputTensor = worker.PeekOutput();
        outputTensor.MakeReadable();
        // Process outputTensor as needed

    }

    private void onDisable()
    {
        worker.Dispose();
    }
}