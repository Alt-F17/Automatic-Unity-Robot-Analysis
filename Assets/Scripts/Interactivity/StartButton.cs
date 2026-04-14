using UnityEngine;
using UnityEngine.SceneManagement;
using TMPro;


public class StartButton : MonoBehaviour
{
    [SerializeField] private string sceneName = "MainScene";
    [SerializeField] private TextMeshProUGUI timerText;

    private bool timerStarted = false;
    private float elapsedTime = 0f;

    private void Update()
    {
        if (!timerStarted)
        {
            return;
        }

        elapsedTime += Time.deltaTime;
        if (timerText != null)
        {
            timerText.text = $"Elapsed Time: {elapsedTime:F2} seconds";
        }
    }

    private bool TryLoadConfiguredScene()
    {
        if (string.IsNullOrWhiteSpace(sceneName))
        {
            Debug.LogError("StartButton: sceneName is empty.");
            return false;
        }

        SceneManager.LoadScene(sceneName);
        return true;
    }

    public void StartSimulation()
    {
        Debug.Log("Start Button Clicked");
        TryLoadConfiguredScene();
    }

    public void ResetGame(){
        Debug.Log("Reset Button Clicked");
        TryLoadConfiguredScene();
    }

    public void StartTimer()
    {
        timerStarted = true;
    }

    public void StopTimer()
    {
        timerStarted = false;
    }

    public void QuitGame()
    {
        Debug.Log("Quit Button Clicked");
        Application.Quit();
    }

}


