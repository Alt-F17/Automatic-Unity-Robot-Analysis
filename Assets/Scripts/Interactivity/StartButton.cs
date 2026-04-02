using UnityEngine;
using Unity.SceneManagement;



public class StartButton : MonoBehaviour
{
    public void StartSimulation()
    {
        Debug.Log("Start Button Clicked");
        SceneManager.LoadScene("MainScene");
    }

    public void QuitGame(){
        Debug.Log("Quit Button Clicked");
        Application.Quit();
    }

}


// draw vectors for force visualization 

