using UnityEngine;
using Unity.SceneManagement;

public class StartButton : MonoBehaviour
{
    private void StartSimulation()
    {
        Debug.Log("Start Button Clicked");
        SceneManager.LoadScene("MainScene");
    }

    private void QuitGame(){
        Debug.Log("Quit Button Clicked");
        Application.Quit();
    }

}


void onDrawGizmos(){
    Gizmos.color = Color.green;
    Gizmos.DrawRay(transform.position, transform.forward * 2);
}