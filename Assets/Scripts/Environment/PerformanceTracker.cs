using UnityEngine;
using System.Collections.Generic;
using System.Linq;

// Tracks performance of all robots and identifies the best performer
// Only the best performer collects detailed physics data

public class PerformanceTracker : MonoBehaviour
{
    [Header("Tracking Settings")]
    [SerializeField] private int rollingWindowSize = 50;  // Last 50 episodes
    [SerializeField] private float updateInterval = 5f;   // Check every 5 seconds
    
    [Header("Info Display")]
    [SerializeField] private bool showDebugInfo = true;
    
    // All robots being tracked
    private List<RobotAgent> allRobots;
    private Dictionary<RobotAgent, RobotPerformance> performanceData;
    
    // Current best performer
    private RobotAgent currentBestRobot;
    private float lastUpdateTime;
    
    // Singleton
    public static PerformanceTracker Instance { get; private set; }
    
    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Destroy(gameObject);
            return; // if exists, dont bother setting it up
        }
        
        allRobots = new List<RobotAgent>();
        performanceData = new Dictionary<RobotAgent, RobotPerformance>();
        lastUpdateTime = Time.time;
    }
    
    void Start()
    {
        // find all robots in the scene
        RobotAgent[] robots = FindObjectsOfType<RobotAgent>();
        
        foreach (RobotAgent robot in robots)
        {
            RegisterRobot(robot);
        }
        
        Debug.Log($"[PerformanceTracker] Registered {allRobots.Count} robots");
        
        // initially, first robot collects data (literally doesnt matter)
        if (allRobots.Count > 0)
        {
            SetBestPerformer(allRobots[0]);
        }
    }
    
    void Update()
    {
        // periodically update who's the best
        if (Time.time - lastUpdateTime > updateInterval)
        {
            UpdateBestPerformer();
            lastUpdateTime = Time.time;
        }
    }
    
    // register a robot for tracking

    public void RegisterRobot(RobotAgent robot)
    {
        if (!allRobots.Contains(robot))
        {
            allRobots.Add(robot);
            performanceData[robot] = new RobotPerformance(rollingWindowSize);
        }
    }

    // record episode result for a robot

    public void RecordEpisode(RobotAgent robot, bool success, float timeTaken, float energyUsed, float accuracy)
    {
        if (!performanceData.ContainsKey(robot))
        {
            RegisterRobot(robot);
        }
        
        performanceData[robot].AddEpisode(success, timeTaken, energyUsed, accuracy);
    }

    // determine which robot is currently performing best

    private void UpdateBestPerformer()
    {
        if (allRobots.Count == 0) return;
        
        RobotAgent bestRobot = null;
        float bestScore = float.MinValue;
        
        foreach (RobotAgent robot in allRobots)
        {
            if (!performanceData.ContainsKey(robot)) continue;
            
            float score = CalculatePerformanceScore(performanceData[robot]);
            
            if (score > bestScore)
            {
                bestScore = score;
                bestRobot = robot;
            }
        }
        
        if (bestRobot != null && bestRobot != currentBestRobot)
        {
            SetBestPerformer(bestRobot); // if doesnt ezist or changes, apply change
        }
    }
    
    // calculate overall performance score
    // where higher is better

    private float CalculatePerformanceScore(RobotPerformance perf)
    {
        if (perf.TotalEpisodes == 0) return 0f;
        
        // Weighted scoring:
        // - Success rate: 50%
        // - Speed (inverse of avg time): 25%
        // - Efficiency (inverse of avg energy): 15%
        // - Accuracy: 10%

        // less intense then ML-Agents training scoring
        // but enough to track best physicd performer on
        // a valid (completed) run
        
        float successScore = perf.SuccessRate * 50f;
        
        float speedScore = 0f;
        if (perf.AverageTime > 0)
        {
            speedScore = (1f / perf.AverageTime) * 25f;
        }
        
        float efficiencyScore = 0f;
        if (perf.AverageEnergy > 0)
        {
            efficiencyScore = (1f / perf.AverageEnergy) * 15f;
        }
        
        float accuracyScore = perf.AverageAccuracy * 10f;
        
        return successScore + speedScore + efficiencyScore + accuracyScore;
    }
    
    // apply changes as mentioned above

    private void SetBestPerformer(RobotAgent robot)
    {
        // disable data collection on old best
        if (currentBestRobot != null)
        {
            DataCollector oldCollector = currentBestRobot.GetComponent<DataCollector>();
            if (oldCollector != null)
            {
                oldCollector.SetCollectDetailedPhysics(false);
            }
        }
        
        // enable on new best
        currentBestRobot = robot;
        DataCollector newCollector = currentBestRobot.GetComponent<DataCollector>();
        if (newCollector != null)
        {
            newCollector.SetCollectDetailedPhysics(true);
        }
        
        string robotName = currentBestRobot.gameObject.name;
        float score = 0f;
        if (performanceData.ContainsKey(robot))
        {
            score = CalculatePerformanceScore(performanceData[robot]);
        }
        
        // visual indicator on the best robot
        Transform magnet = currentBestRobot.transform.Find("BaseRotation/Shoulder/ShoulderJoint/Forearm/ElbowJoint/Magnet");
        if (magnet != null)
        {
            Renderer renderer = magnet.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material.color = Color.green;  // Green = best!
                renderer.material.EnableKeyword("_EMISSION");
                renderer.material.SetColor("_EmissionColor", Color.green);
            }
        }
        
        Debug.Log($"[PerformanceTracker] NEW BEST PERFORMER: {robotName} (Score: {score:F2})");
    }
    
    public RobotAgent GetBestPerformer()
    {
        return currentBestRobot;
    }
    
    public bool IsBestPerformer(RobotAgent robot)
    {
        return robot == currentBestRobot;
    }
    
    void OnGUI()
    // GUI for best performer (right side of screen)
    {
        if (!showDebugInfo || currentBestRobot == null) return;
        
        GUILayout.BeginArea(new Rect(Screen.width - 320, 10, 310, 200));
        
        GUILayout.Label("=== PERFORMANCE TRACKER ===");
        GUILayout.Label($"Best Performer: {currentBestRobot.gameObject.name}");
        
        if (performanceData.ContainsKey(currentBestRobot))
        {
            RobotPerformance perf = performanceData[currentBestRobot];
            GUILayout.Label($"Success Rate: {perf.SuccessRate * 100f:F1}%");
            GUILayout.Label($"Avg Time: {perf.AverageTime:F2}s");
            GUILayout.Label($"Avg Energy: {perf.AverageEnergy:F2}");
            GUILayout.Label($"Avg Accuracy: {perf.AverageAccuracy:F3}");
            GUILayout.Label($"Episodes: {perf.TotalEpisodes}");
            GUILayout.Label($"Score: {CalculatePerformanceScore(perf):F2}");
        }
        
        GUILayout.EndArea();
    }
}

// output performance data for a single robot
public class RobotPerformance
{
    private Queue<EpisodeResult> recentEpisodes;
    private int maxSize;
    
    public int TotalEpisodes { get; private set; }
    public float SuccessRate { get; private set; }
    public float AverageTime { get; private set; }
    public float AverageEnergy { get; private set; }
    public float AverageAccuracy { get; private set; }
    
    public RobotPerformance(int rollingWindowSize)
    {
        recentEpisodes = new Queue<EpisodeResult>();
        maxSize = rollingWindowSize;
        TotalEpisodes = 0;
    }
    
    public void AddEpisode(bool success, float time, float energy, float accuracy)
    {
        // add new episode
        recentEpisodes.Enqueue(new EpisodeResult
        {
            success = success,
            time = time,
            energy = energy,
            accuracy = accuracy
        });
        
        // rem old episodes if over limit
        while (recentEpisodes.Count > maxSize)
        {
            recentEpisodes.Dequeue();
        }
        
        TotalEpisodes++;
        
        // recalc statistics
        UpdateStatistics();
    }
    
    private void UpdateStatistics()
    {
        if (recentEpisodes.Count == 0)
        {
            SuccessRate = 0;
            AverageTime = 0;
            AverageEnergy = 0;
            AverageAccuracy = 0;
            return;
        }
        
        int successCount = 0;
        float totalTime = 0;
        float totalEnergy = 0;
        float totalAccuracy = 0;
        
        foreach (var episode in recentEpisodes)
        {
            if (episode.success) successCount++;
            totalTime += episode.time;
            totalEnergy += episode.energy;
            totalAccuracy += episode.accuracy;
        }
        
        int count = recentEpisodes.Count;
        SuccessRate = (float)successCount / count;
        AverageTime = totalTime / count;
        AverageEnergy = totalEnergy / count;
        AverageAccuracy = totalAccuracy / count;
    }
    
    private struct EpisodeResult
    {
        public bool success;
        public float time;
        public float energy;
        public float accuracy;
    }
}