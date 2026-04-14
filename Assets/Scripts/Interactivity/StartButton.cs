using UnityEngine;
using Unity.SceneManagement;
using TextMeshPro;


public class StartButton : MonoBehaviour
{
    public TextMeshProUGUI timerText;
    private bool timerStarted = false;
    private float elapsedTime = 0f;

    private void Update()
    {
        if (timerStarted)
        {
            elapsedTime += Time.deltaTime;
            timerText.text = "Elapsed Time: " + elapsedTime.ToString("F2") + " seconds";
        }
    }

    public void StartSimulation()
    {
        Debug.Log("Start Button Clicked");
        SceneManager.LoadScene("MainScene");


    }

    public void ResetGame(){
        Debug.Log("Reset Button Clicked");
        SceneManager.LoadScene("MainScene");
    }

    public void StartTimer()
    {
        timerStarted = true;
    }

    public void StopTimer()
    {
        timerStarted = false;
    }

}


