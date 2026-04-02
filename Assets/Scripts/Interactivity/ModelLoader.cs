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

    void Start()
    {
        DontDestroyOnLoad(gameObject);
        if (onnxModel != null)
        {
            runtimeModel = ModelLoader.Load(onnxModel);
            worker =  new Worker(runtimeModel, BackendType.GPUComplete);
            Debug.Log("Model loaded successfully in Awake.");
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

}