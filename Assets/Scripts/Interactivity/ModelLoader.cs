// be sure to install the sentis package from the package manager to use this script
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Sentis;
using Unity.Sentis.Layers;
using System.Runtime.CompilerServices;

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
            Debug.Log("Model loaded successfully in Awake.");
        }
    }
    public IModel GetModel()
    {
        return runtimeModel;
    }

}