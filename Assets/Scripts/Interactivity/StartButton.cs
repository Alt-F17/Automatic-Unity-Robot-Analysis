using UnityEngine;
using UnityEngine.SceneManagement;



public class StartButton : MonoBehaviour
{
    [SerializeField] private string sceneName = "MainScene";

    public void StartSimulation()
    {
        Debug.Log("Start Button Clicked");

        if (string.IsNullOrWhiteSpace(sceneName))
        {
            Debug.LogError("StartButton: sceneName is empty.");
            return;
        }

        SceneManager.LoadScene(sceneName);
    }

    public void QuitGame(){
        Debug.Log("Quit Button Clicked");
        Application.Quit();
    }

}


// draw vectors for force visualization 

