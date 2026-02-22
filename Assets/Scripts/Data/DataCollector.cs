using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using System.Linq;

// Enhanced Data Collector for AURA Project
// Collects comprehensive physics, mechanics, and performance data
// Designed for Probability & Statistics + Mechanics analysis
public class DataCollector : MonoBehaviour
{
    [Header("Data Collection Settings")]
    [SerializeField] private bool collectData = true;
    [SerializeField] private string dataDirectory = "TrainingData";
    [SerializeField] private int maxEpisodesToRecord = 1000;
    [SerializeField] private bool autoExportOnInterval = true;
    [SerializeField] private int exportInterval = 50;

    [Header("Detailed Physics Collection")]
    [SerializeField] private bool exportDetailedPhysics = true;
    [SerializeField] private bool exportInverseKinematics = true;
    [SerializeField] private bool exportStatisticalSummary = true;

    [Header("File Settings")]
    [SerializeField] private string filePrefix = "aura_robot";
    private string currentSessionID;

    // Data storage
    private List<EpisodeData> episodeDataList;
    private List<PhysicsData> detailedPhysicsData;
    private int episodesRecorded;
    private string fullDataPath;

    // Statistics tracking
    private float totalTime;
    private float totalEnergy;
    private int successCount;
    private int failureCount;

    // Advanced statistics for probability analysis
    private List<float> timeSamples;
    private List<float> energySamples;
    private List<float> accuracySamples;

    // Dynamically enable/disable detailed physics collection
    // Called by PerformanceTracker
    public void SetCollectDetailedPhysics(bool enabled)
    {
        exportDetailedPhysics = enabled;
        collectData = enabled;
        if (enabled)
        {
            Debug.Log($"[DataCollector] {gameObject.name} - NOW COLLECTING detailed physics (Best Performer!)");
        }
        else
        {
            Debug.Log($"[DataCollector] {gameObject.name} - Stopped collecting detailed physics");
        }
    }

    void Awake()
    {
        episodeDataList = new List<EpisodeData>();
        detailedPhysicsData = new List<PhysicsData>();
        timeSamples = new List<float>();
        energySamples = new List<float>();
        accuracySamples = new List<float>();
        
        episodesRecorded = 0;
        currentSessionID = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        SetupDataDirectory();
    }

    public void RecordEpisode(float timeTaken, float accuracy, float energyConsumed, bool success, PhysicsData physicsData = null)
    {
        if (!collectData) return;
        if (episodesRecorded >= maxEpisodesToRecord) return;

        // Basic episode data
        EpisodeData data = new EpisodeData
        {
            episodeNumber = episodesRecorded + 1,
            timeTaken = timeTaken,
            accuracy = accuracy,
            energyConsumed = energyConsumed,
            success = success,
            timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss")
        };

        episodeDataList.Add(data);

        // Detailed physics data
        if (physicsData != null && exportDetailedPhysics)
        {
            detailedPhysicsData.Add(physicsData);
        }

        episodesRecorded++;

        // Update statistics
        totalTime += timeTaken;
        totalEnergy += energyConsumed;
        if (success)
            successCount++;
        else
            failureCount++;

        // Collect samples for statistical analysis
        timeSamples.Add(timeTaken);
        energySamples.Add(energyConsumed);
        accuracySamples.Add(accuracy);

        // Auto-export
        if (autoExportOnInterval && episodesRecorded % exportInterval == 0)
        {
            ExportAllData();
        }

        if (episodesRecorded % 10 == 0)
        {
            Debug.Log($"[DataCollector] Recorded {episodesRecorded} episodes. Success: {GetSuccessRate():F2}%");
        }
    }

    // Export all collected data
    public void ExportAllData()
    {
        ExportBasicCSV();
        
        if (exportDetailedPhysics && detailedPhysicsData.Count > 0)
        {
            ExportDetailedPhysicsCSV();
        }
        
        if (exportInverseKinematics && detailedPhysicsData.Count > 0)
        {
            ExportInverseKinematicsCSV();
        }
        
        if (exportStatisticalSummary)
        {
            ExportStatisticalSummary();
            ExportProbabilityDistributions();
        }
    }

    // Export basic episode summary data
    // For: Overall performance analysis, success rate trends
    private void ExportBasicCSV()
    {
        if (episodeDataList.Count == 0) return;

        string filename = $"{filePrefix}_basic_{currentSessionID}.csv";
        string filepath = Path.Combine(fullDataPath, filename);

        try
        {
            using (StreamWriter writer = new StreamWriter(filepath, false))
            {
                // Header
                writer.WriteLine("Episode,Time_Taken,Accuracy,Energy_Consumed,Success,Timestamp");

                // Data rows
                foreach (EpisodeData data in episodeDataList)
                {
                    writer.WriteLine($"{data.episodeNumber},{data.timeTaken:F4},{data.accuracy:F4}," +
                                   $"{data.energyConsumed:F4},{(data.success ? 1 : 0)},{data.timestamp}");
                }
            }

            Debug.Log($"[DataCollector] Basic data exported: {filepath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DataCollector] Failed to export basic data: {e.Message}");
        }
    }

    // Export detailed physics snapshots for each episode
    // For: Mechanical analysis, trajectory analysis, force calculations
    private void ExportDetailedPhysicsCSV()
    {
        string filename = $"{filePrefix}_physics_detailed_{currentSessionID}.csv";
        string filepath = Path.Combine(fullDataPath, filename);

        try
        {
            using (StreamWriter writer = new StreamWriter(filepath, false))
            {
                // Header
                writer.WriteLine("Episode,Timestamp,Base_Angle,Shoulder_Angle,Elbow_Angle," +
                               "Base_Velocity,Shoulder_Velocity,Elbow_Velocity," +
                               "Base_Control,Shoulder_Control,Elbow_Control," +
                               "Magnet_Pos_X,Magnet_Pos_Y,Magnet_Pos_Z," +
                               "Magnet_Vel_X,Magnet_Vel_Y,Magnet_Vel_Z," +
                               "Box_Pos_X,Box_Pos_Y,Box_Pos_Z," +
                               "Box_Vel_X,Box_Vel_Y,Box_Vel_Z," +
                               "Base_Torque, Shoulder_Torque, Elbow_Torque" +
                               "Box_Attached,Energy_Step");

                // Write all snapshots from all episodes
                foreach (PhysicsData episodeData in detailedPhysicsData)
                {
                    foreach (PhysicsSnapshot snapshot in episodeData.snapshots)
                    {
                        writer.WriteLine($"{episodeData.episodeNumber},{snapshot.timestamp:F4}," +
                                       $"{snapshot.baseAngle:F4},{snapshot.shoulderAngle:F4},{snapshot.elbowAngle:F4}," +
                                       $"{snapshot.baseVelocity:F4},{snapshot.shoulderVelocity:F4},{snapshot.elbowVelocity:F4}," +
                                       $"{snapshot.baseControl:F4},{snapshot.shoulderControl:F4},{snapshot.elbowControl:F4}," +
                                       $"{snapshot.magnetPosition.x:F4},{snapshot.magnetPosition.y:F4},{snapshot.magnetPosition.z:F4}," +
                                       $"{snapshot.magnetVelocity.x:F4},{snapshot.magnetVelocity.y:F4},{snapshot.magnetVelocity.z:F4}," +
                                       $"{snapshot.boxPosition.x:F4},{snapshot.boxPosition.y:F4},{snapshot.boxPosition.z:F4}," +
                                       $"{snapshot.boxVelocity.x:F4},{snapshot.boxVelocity.y:F4},{snapshot.boxVelocity.z:F4}," +
                                       $"{snapshot.baseTorque:F4}, {snapshot.shoulderTorque:F4}, {snapshot.elbowTorque:F4}," +
                                       $"{(snapshot.isBoxAttached ? 1 : 0)},{snapshot.energyConsumed:F6}");
                    }
                }
            }

            Debug.Log($"[DataCollector] Detailed physics data exported: {filepath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DataCollector] Failed to export physics data: {e.Message}");
        }
    }

    // Export inverse kinematics data
    // For: IK calculations, workspace analysis, reachability studies
    private void ExportInverseKinematicsCSV()
    {
        string filename = $"{filePrefix}_kinematics_{currentSessionID}.csv";
        string filepath = Path.Combine(fullDataPath, filename);

        try
        {
            using (StreamWriter writer = new StreamWriter(filepath, false))
            {
                // Header
                writer.WriteLine("Episode,Timestamp," +
                               "End_Effector_X,End_Effector_Y,End_Effector_Z," +
                               "Shoulder_Angle,Elbow_Angle,Base_Rotation," +
                               "Reach_Distance,Angle_From_Base," +
                               "Joint_Config_Valid");

                foreach (PhysicsData episodeData in detailedPhysicsData)
                {
                    foreach (PhysicsSnapshot snapshot in episodeData.snapshots)
                    {
                        // Calculate derived kinematics values
                        // just remember that the mass is 1kg 
                        Vector3 magnetPos = snapshot.magnetPosition;
                        Vector2 magnetPos2D = new Vector2(magnetPos.x, magnetPos.z);
                        float reachDistance = magnetPos2D.magnitude;
                        float angleFromBase = Mathf.Atan2(magnetPos.z, magnetPos.x) * Mathf.Rad2Deg;

                        float l1 = Vector3.Distance(shoulderControl.position, elbowControl.position);
                        float l2 = Vector3.Distance(elbowControl.position, magnetPos);

                        float q2 = -Mathf.Acos((magnetPos.x**2 + magnetPos.y**2 - l1**2 - l2**2) / 2 * l1 * l2);
                        float q1 = Mathf.Atan(magnetPos.y / magnetPos.x) + Mathf.Atan((l2 * mathf.sin(q2)) / (l1 + l2*Mathf.cos(q2)));


                        SoftJointLimit linearBaseLimit = baseControl.linearLimit;
                        SoftJointLimit linearShoulderLimit = shoulderControl.linearLimit;
                        SoftJointLimit linearElbowLimit = elbowControl.linearLimit;

                        float ShoulderXOffset = shoulderControl.localPosition.x;
                        
                        // Check if joint configuration is physically valid
                        bool configValid = IsJointConfigurationValid(snapshot.shoulderAngle, snapshot.elbowAngle);

                        writer.WriteLine($"{episodeData.episodeNumber},{snapshot.timestamp:F4}," +
                                       $"{magnetPos.x:F4},{magnetPos.y:F4},{magnetPos.z:F4}," +
                                       $"{snapshot.shoulderAngle:F4},{snapshot.elbowAngle:F4},{snapshot.baseAngle:F4}," +
                                       $"{reachDistance:F4},{angleFromBase:F4}," +
                                       $"{q1}, {q2}" + 
                                       $"{(configValid ? 1 : 0)}");
                    }
                }
            }

            Debug.Log($"[DataCollector] Kinematics data exported: {filepath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DataCollector] Failed to export kinematics data: {e.Message}");
        }
    }

    // Export statistical summary and probability distributions
    // For: Statistical analysis, distribution fitting, hypothesis testing
    private void ExportStatisticalSummary()
    {
        string filename = $"{filePrefix}_statistics_{currentSessionID}.txt";
        string filepath = Path.Combine(fullDataPath, filename);

        try
        {
            using (StreamWriter writer = new StreamWriter(filepath, false))
            {
                writer.WriteLine("=== AURA Robot Training - Statistical Summary ===");
                writer.WriteLine($"Session ID: {currentSessionID}");
                writer.WriteLine($"Generated: {DateTime.Now}");
                writer.WriteLine();

                // === BASIC STATISTICS ===
                writer.WriteLine("--- Overall Performance ---");
                writer.WriteLine($"Total Episodes: {episodesRecorded}");
                writer.WriteLine($"Successful: {successCount}");
                writer.WriteLine($"Failed: {failureCount}");
                writer.WriteLine($"Success Rate: {GetSuccessRate():F4}");
                writer.WriteLine();

                // === CENTRAL TENDENCY ===
                writer.WriteLine("--- Central Tendency Measures ---");
                writer.WriteLine($"Mean Time: {Mean(timeSamples):F4} seconds");
                writer.WriteLine($"Median Time: {Median(timeSamples):F4} seconds");
                writer.WriteLine($"Mode Time: {Mode(timeSamples):F4} seconds (approx)");
                writer.WriteLine();
                writer.WriteLine($"Mean Energy: {Mean(energySamples):F4} units");
                writer.WriteLine($"Median Energy: {Median(energySamples):F4} units");
                writer.WriteLine();
                writer.WriteLine($"Mean Accuracy: {Mean(accuracySamples):F4}");
                writer.WriteLine($"Median Accuracy: {Median(accuracySamples):F4}");
                writer.WriteLine();

                // === DISPERSION ===
                writer.WriteLine("--- Dispersion Measures ---");
                writer.WriteLine($"Time Std Dev: {StandardDeviation(timeSamples):F4}");
                writer.WriteLine($"Time Variance: {Variance(timeSamples):F4}");
                writer.WriteLine($"Time Range: {Range(timeSamples):F4}");
                writer.WriteLine($"Time IQR: {IQR(timeSamples):F4}");
                writer.WriteLine();
                writer.WriteLine($"Energy Std Dev: {StandardDeviation(energySamples):F4}");
                writer.WriteLine($"Energy Variance: {Variance(energySamples):F4}");
                writer.WriteLine($"Energy Range: {Range(energySamples):F4}");
                writer.WriteLine();

                // === DISTRIBUTION SHAPE ===
                writer.WriteLine("--- Distribution Shape ---");
                writer.WriteLine($"Time Skewness: {Skewness(timeSamples):F4}");
                writer.WriteLine($"Time Kurtosis: {Kurtosis(timeSamples):F4}");
                writer.WriteLine();
                writer.WriteLine($"Energy Skewness: {Skewness(energySamples):F4}");
                writer.WriteLine($"Energy Kurtosis: {Kurtosis(energySamples):F4}");
                writer.WriteLine();

                // === PERCENTILES ===
                writer.WriteLine("--- Percentiles ---");
                writer.WriteLine("Time Percentiles:");
                writer.WriteLine($"  25th: {Percentile(timeSamples, 25):F4}");
                writer.WriteLine($"  50th (Median): {Percentile(timeSamples, 50):F4}");
                writer.WriteLine($"  75th: {Percentile(timeSamples, 75):F4}");
                writer.WriteLine($"  90th: {Percentile(timeSamples, 90):F4}");
                writer.WriteLine($"  95th: {Percentile(timeSamples, 95):F4}");
                writer.WriteLine();

                // === BEST PERFORMANCE ===
                writer.WriteLine("--- Best Performance (Successful Episodes Only) ---");
                var successfulTimes = GetSuccessfulSamples(timeSamples);
                var successfulEnergies = GetSuccessfulSamples(energySamples);
                
                if (successfulTimes.Count > 0)
                {
                    writer.WriteLine($"Fastest Time: {successfulTimes.Min():F4} seconds");
                    writer.WriteLine($"Slowest Time: {successfulTimes.Max():F4} seconds");
                    writer.WriteLine($"Lowest Energy: {successfulEnergies.Min():F4} units");
                    writer.WriteLine($"Highest Energy: {successfulEnergies.Max():F4} units");
                    writer.WriteLine($"Best Accuracy: {GetSuccessfulSamples(accuracySamples).Max():F4}");
                }
                writer.WriteLine();

                // === CORRELATION ===
                writer.WriteLine("--- Correlation Analysis ---");
                writer.WriteLine($"Time vs Energy Correlation: {Correlation(timeSamples, energySamples):F4}");
                writer.WriteLine($"Time vs Accuracy Correlation: {Correlation(timeSamples, accuracySamples):F4}");
                writer.WriteLine($"Energy vs Accuracy Correlation: {Correlation(energySamples, accuracySamples):F4}");
                writer.WriteLine();

                // === CONFIDENCE INTERVALS ===
                writer.WriteLine("--- 95% Confidence Intervals ---");
                var timeCI = ConfidenceInterval95(timeSamples);
                var energyCI = ConfidenceInterval95(energySamples);
                writer.WriteLine($"Mean Time: [{timeCI.Item1:F4}, {timeCI.Item2:F4}]");
                writer.WriteLine($"Mean Energy: [{energyCI.Item1:F4}, {energyCI.Item2:F4}]");
            }

            Debug.Log($"[DataCollector] Statistical summary exported: {filepath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DataCollector] Failed to export statistics: {e.Message}");
        }
    }

    // Export probability distribution data for histogram creation
    private void ExportProbabilityDistributions()
    {
        string filename = $"{filePrefix}_distributions_{currentSessionID}.csv";
        string filepath = Path.Combine(fullDataPath, filename);

        try
        {
            using (StreamWriter writer = new StreamWriter(filepath, false))
            {
                writer.WriteLine("Variable,Value,Frequency,Relative_Frequency,Cumulative_Frequency");

                // Time distribution
                var timeFreq = CalculateFrequencyDistribution(timeSamples, 20);
                foreach (var bin in timeFreq)
                {
                    writer.WriteLine($"Time,{bin.Value:F4},{bin.Count},{bin.RelativeFrequency:F4},{bin.CumulativeFrequency:F4}");
                }

                // Energy distribution
                var energyFreq = CalculateFrequencyDistribution(energySamples, 20);
                foreach (var bin in energyFreq)
                {
                    writer.WriteLine($"Energy,{bin.Value:F4},{bin.Count},{bin.RelativeFrequency:F4},{bin.CumulativeFrequency:F4}");
                }

                // Accuracy distribution
                var accuracyFreq = CalculateFrequencyDistribution(accuracySamples, 10);
                foreach (var bin in accuracyFreq)
                {
                    writer.WriteLine($"Accuracy,{bin.Value:F4},{bin.Count},{bin.RelativeFrequency:F4},{bin.CumulativeFrequency:F4}");
                }
            }

            Debug.Log($"[DataCollector] Distribution data exported: {filepath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DataCollector] Failed to export distributions: {e.Message}");
        }
    }

    // === STATISTICAL FUNCTIONS ===

    private float Mean(List<float> samples)
    {
        if (samples.Count == 0) return 0f;
        return samples.Average();
    }

    private float Median(List<float> samples)
    {
        if (samples.Count == 0) return 0f;
        var sorted = samples.OrderBy(x => x).ToList();
        int mid = sorted.Count / 2;
        return sorted.Count % 2 == 0 ? (sorted[mid - 1] + sorted[mid]) / 2f : sorted[mid];
    }

    private float Mode(List<float> samples)
    {
        if (samples.Count == 0) return 0f;
        // Bin the data and find most frequent bin
        var bins = CalculateFrequencyDistribution(samples, 10);
        return bins.OrderByDescending(b => b.Count).First().Value;
    }

    private float StandardDeviation(List<float> samples)
    {
        return Mathf.Sqrt(Variance(samples));
    }

    private float Variance(List<float> samples)
    {
        if (samples.Count < 2) return 0f;
        float mean = Mean(samples);
        return samples.Sum(x => Mathf.Pow(x - mean, 2)) / (samples.Count - 1);
    }

    private float Range(List<float> samples)
    {
        if (samples.Count == 0) return 0f;
        return samples.Max() - samples.Min();
    }

    private float IQR(List<float> samples)
    {
        return Percentile(samples, 75) - Percentile(samples, 25);
    }

    private float Percentile(List<float> samples, float percentile)
    {
        if (samples.Count == 0) return 0f;
        var sorted = samples.OrderBy(x => x).ToList();
        float index = (percentile / 100f) * (sorted.Count - 1);
        int lower = Mathf.FloorToInt(index);
        int upper = Mathf.CeilToInt(index);
        float weight = index - lower;
        return sorted[lower] * (1 - weight) + sorted[upper] * weight;
    }

    private float Skewness(List<float> samples)
    {
        if (samples.Count < 3) return 0f;
        float mean = Mean(samples);
        float stdDev = StandardDeviation(samples);
        if (stdDev == 0) return 0f;
        float n = samples.Count;
        float sum = samples.Sum(x => Mathf.Pow((x - mean) / stdDev, 3));
        return (n / ((n - 1) * (n - 2))) * sum;
    }

    private float Kurtosis(List<float> samples)
    {
        if (samples.Count < 4) return 0f;
        float mean = Mean(samples);
        float stdDev = StandardDeviation(samples);
        if (stdDev == 0) return 0f;
        float n = samples.Count;
        float sum = samples.Sum(x => Mathf.Pow((x - mean) / stdDev, 4));
        return ((n * (n + 1)) / ((n - 1) * (n - 2) * (n - 3))) * sum - (3 * Mathf.Pow(n - 1, 2)) / ((n - 2) * (n - 3));
    }

    private float Correlation(List<float> x, List<float> y)
    {
        if (x.Count != y.Count || x.Count < 2) return 0f;
        float meanX = Mean(x);
        float meanY = Mean(y);
        float stdX = StandardDeviation(x);
        float stdY = StandardDeviation(y);
        if (stdX == 0 || stdY == 0) return 0f;

        float sum = 0f;
        for (int i = 0; i < x.Count; i++)
        {
            sum += (x[i] - meanX) * (y[i] - meanY);
        }
        return sum / ((x.Count - 1) * stdX * stdY);
    }

    private (float, float) ConfidenceInterval95(List<float> samples)
    {
        float mean = Mean(samples);
        float stdError = StandardDeviation(samples) / Mathf.Sqrt(samples.Count);
        float margin = 1.96f * stdError; // z-score for 95% CI
        return (mean - margin, mean + margin);
    }

    private List<FrequencyBin> CalculateFrequencyDistribution(List<float> samples, int numBins)
    {
        if (samples.Count == 0) return new List<FrequencyBin>();

        float min = samples.Min();
        float max = samples.Max();
        float binWidth = (max - min) / numBins;
        if (binWidth == 0) binWidth = 1;

        List<FrequencyBin> bins = new List<FrequencyBin>();
        for (int i = 0; i < numBins; i++)
        {
            float binValue = min + (i + 0.5f) * binWidth;
            int count = samples.Count(x => x >= min + i * binWidth && x < min + (i + 1) * binWidth);
            bins.Add(new FrequencyBin
            {
                Value = binValue,
                Count = count,
                RelativeFrequency = (float)count / samples.Count,
                CumulativeFrequency = 0  // Will be calculated next
            });
        }

        // Calculate cumulative frequency
        float cumulative = 0;
        foreach (var bin in bins)
        {
            cumulative += bin.RelativeFrequency;
            bin.CumulativeFrequency = cumulative;
        }

        return bins;
    }

    private List<float> GetSuccessfulSamples(List<float> allSamples)
    {
        List<float> successful = new List<float>();
        for (int i = 0; i < Mathf.Min(allSamples.Count, episodeDataList.Count); i++)
        {
            if (episodeDataList[i].success)
            {
                successful.Add(allSamples[i]);
            }
        }
        return successful;
    }

    private bool IsJointConfigurationValid(float shoulderAngle, float elbowAngle)
    {
        // Check if angles are within typical robot joint limits
        return shoulderAngle >= -90f && shoulderAngle <= 180f && elbowAngle >= -150f && elbowAngle <= 0f;
    }

    private void SetupDataDirectory()
    {
        string projectPath = Application.dataPath;
        fullDataPath = Path.Combine(Directory.GetParent(projectPath).FullName, dataDirectory);

        if (!Directory.Exists(fullDataPath))
        {
            Directory.CreateDirectory(fullDataPath);
            Debug.Log($"[DataCollector] Created data directory: {fullDataPath}");
        }
    }

    private float GetSuccessRate()
    {
        if (episodesRecorded == 0) return 0f;
        return (successCount / (float)episodesRecorded) * 100f;
    }

    void OnApplicationQuit()
    {
        if (collectData && episodeDataList.Count > 0)
        {
            ExportAllData();
            Debug.Log("[DataCollector] Final data export complete.");
        }
    }

    [ContextMenu("Export Data Now")]
    public void ManualExport()
    {
        ExportAllData();
    }

    [ContextMenu("Clear All Data")]
    public void ClearData()
    {
        episodeDataList.Clear();
        detailedPhysicsData.Clear();
        timeSamples.Clear();
        energySamples.Clear();
        accuracySamples.Clear();
        episodesRecorded = 0;
        totalTime = 0;
        totalEnergy = 0;
        successCount = 0;
        failureCount = 0;
        Debug.Log("[DataCollector] All data cleared.");
    }
}

// === DATA STRUCTURES ===

[System.Serializable]
public class EpisodeData
{
    public int episodeNumber;
    public float timeTaken;
    public float accuracy;
    public float energyConsumed;
    public bool success;
    public string timestamp;
}

[System.Serializable]
public class PhysicsData
{
    public int episodeNumber;
    public float startTime;
    public float endTime;
    public float timeTaken;
    public bool success;
    public float finalAccuracy;
    public float totalEnergyConsumed;
    public List<PhysicsSnapshot> snapshots = new List<PhysicsSnapshot>();
}

[System.Serializable]
public class PhysicsSnapshot
{
    public float timestamp;
    
    // Joint angles (degrees)
    public float baseAngle;
    public float shoulderAngle;
    public float elbowAngle;
    
    // Joint velocities (deg/s)
    public float baseVelocity;
    public float shoulderVelocity;
    public float elbowVelocity;
    
    // Control inputs (-1 to 1)
    public float baseControl;
    public float shoulderControl;
    public float elbowControl;
    
    // End effector state
    public Vector3 magnetPosition;
    public Vector3 magnetVelocity;
    
    // Box state
    public Vector3 boxPosition;
    public Vector3 boxVelocity;
    public bool isBoxAttached;

    // Joint torque
    public float baseTorque;

    public float shoulderTorque;

    public float elbowTorque;

    private float GetJointTorque(ArticulationBody joint)
    {
        if(joint == null)
        {
            return 0f;
        }
        if(joint.jointForce.dofCount == 0)
        {
            return 0f;
        }
        return joint.jointForce[0];
    }
    
    // Energy
    public float energyConsumed;
}

public class FrequencyBin
{
    public float Value;
    public int Count;
    public float RelativeFrequency;
    public float CumulativeFrequency;
}