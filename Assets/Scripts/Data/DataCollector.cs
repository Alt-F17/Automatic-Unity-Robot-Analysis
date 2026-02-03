using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace AutomaticUnityRobotAnalysis
{
    // Data collector script for future analysis and training
    public class DataCollector : MonoBehaviour
    {
        #region Serialized Fields

        [Header("Data Collection Settings")]
        [SerializeField] private bool collectData = true;
        [SerializeField] private string dataDirectory = "TrainingData";
        [SerializeField] private int maxEpisodesToRecord = 1000;
        [SerializeField] private bool autoExportOnInterval = true;
        [SerializeField] private int exportInterval = 50;

        [Header("Detailed Data Export Options")]
        [SerializeField] private bool exportDetailedPhysics = true;
        [SerializeField] private bool exportInverseKinematics = true;
        [SerializeField] private bool exportStatisticalSummary = true;
        [SerializeField] private bool exportProbabilityDistributions = true;

        [Header("Snapshot Settings")]
        [Tooltip("How often to capture physics snapshots during an episode (in seconds)")]
        [SerializeField] private float snapshotInterval = 0.1f;

        [Header("File Settings")]
        [SerializeField] private string filePrefix = "aura_robot";

        #endregion

        #region Private Fields

        private string currentSessionID;
        private string fullDataPath;

        // Data storage
        private List<EpisodeData> episodeDataList;
        private List<PhysicsData> detailedPhysicsData;
        private PhysicsData currentEpisodePhysics;
        private int episodesRecorded;

        // Statistics tracking
        private float totalTime;
        private float totalEnergy;
        private int successCount;
        private int failureCount;

        // Advanced statistics for probability analysis
        private List<float> timeSamples;
        private List<float> energySamples;
        private List<float> accuracySamples;
        private List<float> distanceSamples;

        // Snapshot timing
        private float lastSnapshotTime;

        #endregion

        private void Awake() // init method
        {
            InitializeCollections();
            currentSessionID = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            SetupDataDirectory();
        }

        private void OnApplicationQuit() // fail-safe export on exit
        {
            if (collectData && episodeDataList.Count > 0)
            {
                ExportAllData();
                Debug.Log("[DataCollector] Final data export complete.");
            }
        }

        #region Initialization

        private void InitializeCollections()
        {
            episodeDataList = new List<EpisodeData>();
            detailedPhysicsData = new List<PhysicsData>();
            timeSamples = new List<float>();
            energySamples = new List<float>();
            accuracySamples = new List<float>();
            distanceSamples = new List<float>();
            episodesRecorded = 0;
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

        #endregion

        #region Episode Recording API

        /// <summary>
        /// Call this at the start of each episode to begin recording physics data
        /// </summary>
        public void BeginEpisodeRecording(int episodeNumber)
        {
            if (!collectData) return;

            currentEpisodePhysics = new PhysicsData
            {
                episodeNumber = episodeNumber,
                startTime = Time.time,
                snapshots = new List<PhysicsSnapshot>()
            };
            lastSnapshotTime = Time.time;
        }

        /// <summary>
        /// Record a physics snapshot during the episode
        /// Call this from FixedUpdate or at regular intervals
        /// </summary>
        public void RecordPhysicsSnapshot(
            float baseAngle, float shoulderAngle, float elbowAngle,
            float baseVelocity, float shoulderVelocity, float elbowVelocity,
            float baseControl, float shoulderControl, float elbowControl,
            Vector3 footPosition, Vector3 footVelocity,
            Vector3 boxPosition, Vector3 boxVelocity,
            bool isBoxAttached, float energyConsumed)
        {
            if (!collectData || currentEpisodePhysics == null) return;
            if (Time.time - lastSnapshotTime < snapshotInterval) return;

            var snapshot = new PhysicsSnapshot
            {
                timestamp = Time.time - currentEpisodePhysics.startTime,
                baseAngle = baseAngle,
                shoulderAngle = shoulderAngle,
                elbowAngle = elbowAngle,
                baseVelocity = baseVelocity,
                shoulderVelocity = shoulderVelocity,
                elbowVelocity = elbowVelocity,
                baseControl = baseControl,
                shoulderControl = shoulderControl,
                elbowControl = elbowControl,
                footPosition = footPosition,
                footVelocity = footVelocity,
                boxPosition = boxPosition,
                boxVelocity = boxVelocity,
                isBoxAttached = isBoxAttached,
                energyConsumed = energyConsumed
            };

            currentEpisodePhysics.snapshots.Add(snapshot);
            lastSnapshotTime = Time.time;
        }

        /// <summary>
        /// Record the completion of an episode with all relevant metrics
        /// </summary>
        public void RecordEpisode(float timeTaken, float accuracy, float energyConsumed, 
            bool success, float finalDistance = 0f)
        {
            if (!collectData) return;
            if (episodesRecorded >= maxEpisodesToRecord) return;

            // Basic episode data
            var data = new EpisodeData
            {
                episodeNumber = episodesRecorded + 1,
                timeTaken = timeTaken,
                accuracy = accuracy,
                energyConsumed = energyConsumed,
                success = success,
                finalDistance = finalDistance,
                timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss")
            };

            episodeDataList.Add(data);

            // Finalize physics data for this episode
            if (currentEpisodePhysics != null && exportDetailedPhysics)
            {
                currentEpisodePhysics.endTime = Time.time;
                currentEpisodePhysics.timeTaken = timeTaken;
                currentEpisodePhysics.success = success;
                currentEpisodePhysics.finalAccuracy = accuracy;
                currentEpisodePhysics.totalEnergyConsumed = energyConsumed;
                detailedPhysicsData.Add(currentEpisodePhysics);
                currentEpisodePhysics = null;
            }

            episodesRecorded++;

            // Update aggregate statistics
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
            distanceSamples.Add(finalDistance);

            // Auto-export at intervals
            if (autoExportOnInterval && episodesRecorded % exportInterval == 0)
            {
                ExportAllData();
            }

            // Log progress
            if (episodesRecorded % 10 == 0)
            {
                Debug.Log($"[DataCollector] Recorded {episodesRecorded} episodes. " +
                         $"Success Rate: {GetSuccessRate():F2}%");
            }
        }

        #endregion

        #region Data Export

        /// <summary>
        /// Export all collected data to CSV and text files
        /// </summary>
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

            if (exportStatisticalSummary && episodeDataList.Count > 0)
            {
                ExportStatisticalSummary();
            }

            if (exportProbabilityDistributions && timeSamples.Count > 0)
            {
                ExportProbabilityDistributions();
            }

            Debug.Log($"[DataCollector] Data export complete. Session: {currentSessionID}");
        }

        /// <summary>
        /// Export basic episode summary data
        /// </summary>
        private void ExportBasicCSV()
        {
            if (episodeDataList.Count == 0) return;

            string filename = $"{filePrefix}_episodes_{currentSessionID}.csv";
            string filepath = Path.Combine(fullDataPath, filename);

            try
            {
                using (var writer = new StreamWriter(filepath, false))
                {
                    writer.WriteLine("Episode,Time_Taken,Accuracy,Energy_Consumed,Final_Distance,Success,Timestamp");

                    foreach (var data in episodeDataList)
                    {
                        writer.WriteLine($"{data.episodeNumber}," +
                                       $"{data.timeTaken:F4}," +
                                       $"{data.accuracy:F4}," +
                                       $"{data.energyConsumed:F4}," +
                                       $"{data.finalDistance:F4}," +
                                       $"{(data.success ? 1 : 0)}," +
                                       $"{data.timestamp}");
                    }
                }

                Debug.Log($"[DataCollector] Basic data exported: {filepath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[DataCollector] Failed to export basic data: {e.Message}");
            }
        }

        /// <summary>
        /// Export detailed physics snapshots for trajectory and force analysis
        /// </summary>
        private void ExportDetailedPhysicsCSV()
        {
            string filename = $"{filePrefix}_physics_detailed_{currentSessionID}.csv";
            string filepath = Path.Combine(fullDataPath, filename);

            try
            {
                using (var writer = new StreamWriter(filepath, false))
                {
                    writer.WriteLine("Episode,Timestamp,Base_Angle,Shoulder_Angle,Elbow_Angle," +
                                   "Base_Velocity,Shoulder_Velocity,Elbow_Velocity," +
                                   "Base_Control,Shoulder_Control,Elbow_Control," +
                                   "Foot_Pos_X,Foot_Pos_Y,Foot_Pos_Z," +
                                   "Foot_Vel_X,Foot_Vel_Y,Foot_Vel_Z," +
                                   "Box_Pos_X,Box_Pos_Y,Box_Pos_Z," +
                                   "Box_Vel_X,Box_Vel_Y,Box_Vel_Z," +
                                   "Box_Attached,Energy_Step");

                    foreach (var episodeData in detailedPhysicsData)
                    {
                        foreach (var snapshot in episodeData.snapshots)
                        {
                            writer.WriteLine($"{episodeData.episodeNumber}," +
                                           $"{snapshot.timestamp:F4}," +
                                           $"{snapshot.baseAngle:F4}," +
                                           $"{snapshot.shoulderAngle:F4}," +
                                           $"{snapshot.elbowAngle:F4}," +
                                           $"{snapshot.baseVelocity:F4}," +
                                           $"{snapshot.shoulderVelocity:F4}," +
                                           $"{snapshot.elbowVelocity:F4}," +
                                           $"{snapshot.baseControl:F4}," +
                                           $"{snapshot.shoulderControl:F4}," +
                                           $"{snapshot.elbowControl:F4}," +
                                           $"{snapshot.footPosition.x:F4}," +
                                           $"{snapshot.footPosition.y:F4}," +
                                           $"{snapshot.footPosition.z:F4}," +
                                           $"{snapshot.footVelocity.x:F4}," +
                                           $"{snapshot.footVelocity.y:F4}," +
                                           $"{snapshot.footVelocity.z:F4}," +
                                           $"{snapshot.boxPosition.x:F4}," +
                                           $"{snapshot.boxPosition.y:F4}," +
                                           $"{snapshot.boxPosition.z:F4}," +
                                           $"{snapshot.boxVelocity.x:F4}," +
                                           $"{snapshot.boxVelocity.y:F4}," +
                                           $"{snapshot.boxVelocity.z:F4}," +
                                           $"{(snapshot.isBoxAttached ? 1 : 0)}," +
                                           $"{snapshot.energyConsumed:F4}");
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

        /// <summary>
        /// Export inverse kinematics data for workspace and reachability analysis
        /// </summary>
        private void ExportInverseKinematicsCSV()
        {
            string filename = $"{filePrefix}_kinematics_{currentSessionID}.csv";
            string filepath = Path.Combine(fullDataPath, filename);

            try
            {
                using (var writer = new StreamWriter(filepath, false))
                {
                    writer.WriteLine("Episode,Timestamp," +
                                   "End_Effector_X,End_Effector_Y,End_Effector_Z," +
                                   "Shoulder_Angle,Elbow_Angle,Base_Rotation," +
                                   "Reach_Distance,Angle_From_Base," +
                                   "Joint_Config_Valid");

                    foreach (var episodeData in detailedPhysicsData)
                    {
                        foreach (var snapshot in episodeData.snapshots)
                        {
                            float reachDistance = Mathf.Sqrt(
                                snapshot.footPosition.x * snapshot.footPosition.x +
                                snapshot.footPosition.z * snapshot.footPosition.z);
                            float angleFromBase = Mathf.Atan2(snapshot.footPosition.z, 
                                snapshot.footPosition.x) * Mathf.Rad2Deg;
                            bool validConfig = IsJointConfigurationValid(
                                snapshot.shoulderAngle, snapshot.elbowAngle);

                            writer.WriteLine($"{episodeData.episodeNumber}," +
                                           $"{snapshot.timestamp:F4}," +
                                           $"{snapshot.footPosition.x:F4}," +
                                           $"{snapshot.footPosition.y:F4}," +
                                           $"{snapshot.footPosition.z:F4}," +
                                           $"{snapshot.shoulderAngle:F4}," +
                                           $"{snapshot.elbowAngle:F4}," +
                                           $"{snapshot.baseAngle:F4}," +
                                           $"{reachDistance:F4}," +
                                           $"{angleFromBase:F4}," +
                                           $"{(validConfig ? 1 : 0)}");
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

        /// <summary>
        /// Export comprehensive statistical summary
        /// </summary>
        private void ExportStatisticalSummary()
        {
            string filename = $"{filePrefix}_statistics_{currentSessionID}.txt";
            string filepath = Path.Combine(fullDataPath, filename);

            try
            {
                using (var writer = new StreamWriter(filepath, false))
                {
                    writer.WriteLine("═══════════════════════════════════════════════════════════════");
                    writer.WriteLine("           AURA Robot Training - Statistical Summary           ");
                    writer.WriteLine("═══════════════════════════════════════════════════════════════");
                    writer.WriteLine($"Session ID: {currentSessionID}");
                    writer.WriteLine($"Generated: {DateTime.Now}");
                    writer.WriteLine();

                    // Overall Performance
                    writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                    writer.WriteLine("│                    OVERALL PERFORMANCE                      │");
                    writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                    writer.WriteLine($"  Total Episodes:    {episodesRecorded}");
                    writer.WriteLine($"  Successful:        {successCount}");
                    writer.WriteLine($"  Failed:            {failureCount}");
                    writer.WriteLine($"  Success Rate:      {GetSuccessRate():F2}%");
                    writer.WriteLine($"  Avg Time/Episode:  {(episodesRecorded > 0 ? totalTime / episodesRecorded : 0):F4} seconds");
                    writer.WriteLine($"  Avg Energy/Ep:     {(episodesRecorded > 0 ? totalEnergy / episodesRecorded : 0):F4} units");
                    writer.WriteLine();

                    // Central Tendency
                    writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                    writer.WriteLine("│                   CENTRAL TENDENCY                          │");
                    writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                    writer.WriteLine("  Time (seconds):");
                    writer.WriteLine($"    Mean:   {Mean(timeSamples):F4}");
                    writer.WriteLine($"    Median: {Median(timeSamples):F4}");
                    writer.WriteLine($"    Mode:   {Mode(timeSamples):F4} (approx)");
                    writer.WriteLine();
                    writer.WriteLine("  Energy (units):");
                    writer.WriteLine($"    Mean:   {Mean(energySamples):F4}");
                    writer.WriteLine($"    Median: {Median(energySamples):F4}");
                    writer.WriteLine();
                    writer.WriteLine("  Accuracy:");
                    writer.WriteLine($"    Mean:   {Mean(accuracySamples):F4}");
                    writer.WriteLine($"    Median: {Median(accuracySamples):F4}");
                    writer.WriteLine();

                    // Dispersion
                    writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                    writer.WriteLine("│                     DISPERSION                              │");
                    writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                    writer.WriteLine("  Time:");
                    writer.WriteLine($"    Std Dev:  {StandardDeviation(timeSamples):F4}");
                    writer.WriteLine($"    Variance: {Variance(timeSamples):F4}");
                    writer.WriteLine($"    Range:    {Range(timeSamples):F4}");
                    writer.WriteLine($"    IQR:      {IQR(timeSamples):F4}");
                    writer.WriteLine();
                    writer.WriteLine("  Energy:");
                    writer.WriteLine($"    Std Dev:  {StandardDeviation(energySamples):F4}");
                    writer.WriteLine($"    Variance: {Variance(energySamples):F4}");
                    writer.WriteLine($"    Range:    {Range(energySamples):F4}");
                    writer.WriteLine();

                    // Distribution Shape
                    writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                    writer.WriteLine("│                  DISTRIBUTION SHAPE                         │");
                    writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                    writer.WriteLine("  Time:");
                    writer.WriteLine($"    Skewness: {Skewness(timeSamples):F4}");
                    writer.WriteLine($"    Kurtosis: {Kurtosis(timeSamples):F4}");
                    writer.WriteLine();
                    writer.WriteLine("  Energy:");
                    writer.WriteLine($"    Skewness: {Skewness(energySamples):F4}");
                    writer.WriteLine($"    Kurtosis: {Kurtosis(energySamples):F4}");
                    writer.WriteLine();

                    // Percentiles
                    writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                    writer.WriteLine("│                     PERCENTILES                             │");
                    writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                    writer.WriteLine("  Time Percentiles:");
                    writer.WriteLine($"    10th: {Percentile(timeSamples, 10):F4}");
                    writer.WriteLine($"    25th: {Percentile(timeSamples, 25):F4}");
                    writer.WriteLine($"    50th: {Percentile(timeSamples, 50):F4}");
                    writer.WriteLine($"    75th: {Percentile(timeSamples, 75):F4}");
                    writer.WriteLine($"    90th: {Percentile(timeSamples, 90):F4}");
                    writer.WriteLine($"    95th: {Percentile(timeSamples, 95):F4}");
                    writer.WriteLine();

                    // Best Performance (Successful Episodes)
                    var successfulTimes = GetSuccessfulSamples(timeSamples);
                    var successfulEnergies = GetSuccessfulSamples(energySamples);
                    var successfulAccuracies = GetSuccessfulSamples(accuracySamples);

                    if (successfulTimes.Count > 0)
                    {
                        writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                        writer.WriteLine("│              BEST PERFORMANCE (Successful)                  │");
                        writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                        writer.WriteLine($"  Fastest Time:    {successfulTimes.Min():F4} seconds");
                        writer.WriteLine($"  Slowest Time:    {successfulTimes.Max():F4} seconds");
                        writer.WriteLine($"  Lowest Energy:   {successfulEnergies.Min():F4} units");
                        writer.WriteLine($"  Highest Energy:  {successfulEnergies.Max():F4} units");
                        writer.WriteLine($"  Best Accuracy:   {successfulAccuracies.Max():F4}");
                        writer.WriteLine();
                    }

                    // Correlation Analysis
                    writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                    writer.WriteLine("│                 CORRELATION ANALYSIS                        │");
                    writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                    writer.WriteLine($"  Time vs Energy:     {Correlation(timeSamples, energySamples):F4}");
                    writer.WriteLine($"  Time vs Accuracy:   {Correlation(timeSamples, accuracySamples):F4}");
                    writer.WriteLine($"  Energy vs Accuracy: {Correlation(energySamples, accuracySamples):F4}");
                    writer.WriteLine();

                    // Confidence Intervals
                    var timeCI = ConfidenceInterval95(timeSamples);
                    var energyCI = ConfidenceInterval95(energySamples);
                    var accuracyCI = ConfidenceInterval95(accuracySamples);

                    writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                    writer.WriteLine("│              95% CONFIDENCE INTERVALS                       │");
                    writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                    writer.WriteLine($"  Mean Time:     [{timeCI.lower:F4}, {timeCI.upper:F4}]");
                    writer.WriteLine($"  Mean Energy:   [{energyCI.lower:F4}, {energyCI.upper:F4}]");
                    writer.WriteLine($"  Mean Accuracy: [{accuracyCI.lower:F4}, {accuracyCI.upper:F4}]");
                    writer.WriteLine();

                    // Learning Progress
                    if (episodeDataList.Count >= 20)
                    {
                        writer.WriteLine("┌─────────────────────────────────────────────────────────────┐");
                        writer.WriteLine("│                  LEARNING PROGRESS                          │");
                        writer.WriteLine("└─────────────────────────────────────────────────────────────┘");
                        
                        int quarterSize = episodeDataList.Count / 4;
                        var q1Success = episodeDataList.Take(quarterSize).Count(e => e.success) / (float)quarterSize * 100;
                        var q4Success = episodeDataList.Skip(3 * quarterSize).Count(e => e.success) / (float)quarterSize * 100;
                        
                        writer.WriteLine($"  First Quarter Success Rate:  {q1Success:F2}%");
                        writer.WriteLine($"  Last Quarter Success Rate:   {q4Success:F2}%");
                        writer.WriteLine($"  Improvement:                 {(q4Success - q1Success):+0.00;-0.00}%");
                    }

                    writer.WriteLine();
                    writer.WriteLine("═══════════════════════════════════════════════════════════════");
                }

                Debug.Log($"[DataCollector] Statistical summary exported: {filepath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[DataCollector] Failed to export statistics: {e.Message}");
            }
        }

        /// <summary>
        /// Export probability distribution data for histogram creation
        /// </summary>
        private void ExportProbabilityDistributions()
        {
            string filename = $"{filePrefix}_distributions_{currentSessionID}.csv";
            string filepath = Path.Combine(fullDataPath, filename);

            try
            {
                using (var writer = new StreamWriter(filepath, false))
                {
                    writer.WriteLine("Variable,Bin_Center,Count,Relative_Frequency,Cumulative_Frequency");

                    // Time distribution (20 bins)
                    var timeFreq = CalculateFrequencyDistribution(timeSamples, 20);
                    foreach (var bin in timeFreq)
                    {
                        writer.WriteLine($"Time,{bin.BinCenter:F4},{bin.Count},{bin.RelativeFrequency:F6},{bin.CumulativeFrequency:F6}");
                    }

                    // Energy distribution (20 bins)
                    var energyFreq = CalculateFrequencyDistribution(energySamples, 20);
                    foreach (var bin in energyFreq)
                    {
                        writer.WriteLine($"Energy,{bin.BinCenter:F4},{bin.Count},{bin.RelativeFrequency:F6},{bin.CumulativeFrequency:F6}");
                    }

                    // Accuracy distribution (10 bins)
                    var accuracyFreq = CalculateFrequencyDistribution(accuracySamples, 10);
                    foreach (var bin in accuracyFreq)
                    {
                        writer.WriteLine($"Accuracy,{bin.BinCenter:F4},{bin.Count},{bin.RelativeFrequency:F6},{bin.CumulativeFrequency:F6}");
                    }

                    // Distance distribution (15 bins)
                    if (distanceSamples.Count > 0)
                    {
                        var distanceFreq = CalculateFrequencyDistribution(distanceSamples, 15);
                        foreach (var bin in distanceFreq)
                        {
                            writer.WriteLine($"Distance,{bin.BinCenter:F4},{bin.Count},{bin.RelativeFrequency:F6},{bin.CumulativeFrequency:F6}");
                        }
                    }
                }

                Debug.Log($"[DataCollector] Probability distributions exported: {filepath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[DataCollector] Failed to export distributions: {e.Message}");
            }
        }

        #endregion

        #region Statistical Functions

        private float Mean(List<float> samples)
        {
            if (samples == null || samples.Count == 0) return 0f;
            return samples.Average();
        }

        private float Median(List<float> samples)
        {
            if (samples == null || samples.Count == 0) return 0f;
            var sorted = samples.OrderBy(x => x).ToList();
            int mid = sorted.Count / 2;
            return sorted.Count % 2 == 0 
                ? (sorted[mid - 1] + sorted[mid]) / 2f 
                : sorted[mid];
        }

        private float Mode(List<float> samples)
        {
            if (samples == null || samples.Count == 0) return 0f;
            var bins = CalculateFrequencyDistribution(samples, 10);
            return bins.Count > 0 
                ? bins.OrderByDescending(b => b.Count).First().BinCenter 
                : 0f;
        }

        private float StandardDeviation(List<float> samples)
        {
            return Mathf.Sqrt(Variance(samples));
        }

        private float Variance(List<float> samples)
        {
            if (samples == null || samples.Count < 2) return 0f;
            float mean = Mean(samples);
            return samples.Sum(x => Mathf.Pow(x - mean, 2)) / (samples.Count - 1);
        }

        private float Range(List<float> samples)
        {
            if (samples == null || samples.Count == 0) return 0f;
            return samples.Max() - samples.Min();
        }

        private float IQR(List<float> samples)
        {
            return Percentile(samples, 75) - Percentile(samples, 25);
        }

        private float Percentile(List<float> samples, float percentile)
        {
            if (samples == null || samples.Count == 0) return 0f;
            var sorted = samples.OrderBy(x => x).ToList();
            float index = (percentile / 100f) * (sorted.Count - 1);
            int lower = Mathf.FloorToInt(index);
            int upper = Mathf.Min(Mathf.CeilToInt(index), sorted.Count - 1);
            float weight = index - lower;
            return sorted[lower] * (1 - weight) + sorted[upper] * weight;
        }

        private float Skewness(List<float> samples)
        {
            if (samples == null || samples.Count < 3) return 0f;
            float mean = Mean(samples);
            float stdDev = StandardDeviation(samples);
            if (stdDev == 0) return 0f;
            
            float n = samples.Count;
            float sum = samples.Sum(x => Mathf.Pow((x - mean) / stdDev, 3));
            return (n / ((n - 1) * (n - 2))) * sum;
        }

        private float Kurtosis(List<float> samples)
        {
            if (samples == null || samples.Count < 4) return 0f;
            float mean = Mean(samples);
            float stdDev = StandardDeviation(samples);
            if (stdDev == 0) return 0f;
            
            float n = samples.Count;
            float sum = samples.Sum(x => Mathf.Pow((x - mean) / stdDev, 4));
            return ((n * (n + 1)) / ((n - 1) * (n - 2) * (n - 3))) * sum 
                   - (3 * Mathf.Pow(n - 1, 2)) / ((n - 2) * (n - 3));
        }

        private float Correlation(List<float> x, List<float> y)
        {
            if (x == null || y == null || x.Count != y.Count || x.Count < 2) return 0f;
            
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

        private (float lower, float upper) ConfidenceInterval95(List<float> samples)
        {
            if (samples == null || samples.Count == 0) return (0f, 0f);
            
            float mean = Mean(samples);
            float stdError = StandardDeviation(samples) / Mathf.Sqrt(samples.Count);
            float margin = 1.96f * stdError; // z-score for 95% CI
            return (mean - margin, mean + margin);
        }

        private List<FrequencyBin> CalculateFrequencyDistribution(List<float> samples, int numBins)
        {
            var bins = new List<FrequencyBin>();
            if (samples == null || samples.Count == 0) return bins;

            float min = samples.Min();
            float max = samples.Max();
            float binWidth = (max - min) / numBins;
            
            if (binWidth == 0) binWidth = 1;

            for (int i = 0; i < numBins; i++)
            {
                float binStart = min + i * binWidth;
                float binEnd = binStart + binWidth;
                float binCenter = (binStart + binEnd) / 2f;

                int count = i == numBins - 1
                    ? samples.Count(v => v >= binStart && v <= binEnd)
                    : samples.Count(v => v >= binStart && v < binEnd);

                bins.Add(new FrequencyBin
                {
                    BinCenter = binCenter,
                    Count = count,
                    RelativeFrequency = count / (float)samples.Count
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
            var successful = new List<float>();
            int count = Mathf.Min(allSamples.Count, episodeDataList.Count);
            
            for (int i = 0; i < count; i++)
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
            return shoulderAngle >= -90f && shoulderAngle <= 180f 
                && elbowAngle >= -150f && elbowAngle <= 150f;
        }

        #endregion

        #region Utility Methods

        public float GetSuccessRate()
        {
            if (episodesRecorded == 0) return 0f;
            return (successCount / (float)episodesRecorded) * 100f;
        }

        public int GetEpisodesRecorded() => episodesRecorded;
        public int GetSuccessCount() => successCount;
        public int GetFailureCount() => failureCount;
        public bool IsCollecting => collectData && episodesRecorded < maxEpisodesToRecord;

        /// <summary>
        /// Enable or disable detailed physics data collection
        /// Called by PerformanceTracker to focus data collection on the best performer
        /// </summary>
        public void SetCollectDetailedPhysics(bool enabled)
        {
            exportDetailedPhysics = enabled;
            Debug.Log($"[DataCollector] Detailed physics collection: {(enabled ? "ENABLED" : "DISABLED")}");
        }

        #endregion

        #region Context Menu Actions

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
            distanceSamples.Clear();
            currentEpisodePhysics = null;
            episodesRecorded = 0;
            totalTime = 0;
            totalEnergy = 0;
            successCount = 0;
            failureCount = 0;
            Debug.Log("[DataCollector] All data cleared.");
        }

        [ContextMenu("Start New Session")]
        public void StartNewSession()
        {
            ClearData();
            currentSessionID = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            Debug.Log($"[DataCollector] New session started: {currentSessionID}");
        }

        #endregion
    }

    #region Data Structures

    [Serializable]
    public class EpisodeData
    {
        public int episodeNumber;
        public float timeTaken;
        public float accuracy;
        public float energyConsumed;
        public float finalDistance;
        public bool success;
        public string timestamp;
    }

    [Serializable]
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

    [Serializable]
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
        public Vector3 footPosition;
        public Vector3 footVelocity;

        // Box state
        public Vector3 boxPosition;
        public Vector3 boxVelocity;
        public bool isBoxAttached;

        // Energy
        public float energyConsumed;
    }

    public class FrequencyBin
    {
        public float BinCenter;
        public int Count;
        public float RelativeFrequency;
        public float CumulativeFrequency;
    }

    #endregion
}
