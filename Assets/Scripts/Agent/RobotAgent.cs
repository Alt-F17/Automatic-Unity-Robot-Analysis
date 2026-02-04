using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// 2-Joint + Rotation Robotic Arm with Magnetic Pickup
/// Structure: Base Rotation → Shoulder (upper arm) → Elbow → Forearm (lower arm) → Magnet
/// Automatically picks up box when magnet enters collision range
/// </summary>
public class RobotAgent : Agent
{
    [Header("Robot Joint Components")]
    [SerializeField] private ArticulationBody baseRotation;    // Rotates entire arm (Y-axis)
    [SerializeField] private ArticulationBody shoulderJoint;   // Shoulder joint
    [SerializeField] private ArticulationBody elbowJoint;      // Elbow joint
    [SerializeField] private Transform magnet;                 // End effector with magnet

    [Header("Environment Objects")]
    [SerializeField] private Rigidbody movableBox;
    [SerializeField] private Transform targetZoneA;            // Starting zone
    [SerializeField] private Transform targetZoneB;            // Goal zone
    [SerializeField] private Transform floor;

    [Header("Magnet Settings")]
    [SerializeField] private float magneticRange = 0.5f;       // Distance to auto-pickup
    [SerializeField] private float magneticStrength = 100f;    // Spring force strength
    [SerializeField] private bool visualizeMagnetRange = true;

    [Header("Training Parameters")]
    [SerializeField] private float maxMotorForce = 100f;
    [SerializeField] private float movementSpeed = 50f;
    [SerializeField] private float rewardMultiplier = 1f;

    [Header("Power Budget System")]
    [SerializeField] private bool usePowerBudget = false;
    [SerializeField] private float maxPowerBudget = 100f;
    [SerializeField] private float currentPower = 100f;

    [Header("Random Position System")]
    [SerializeField] private bool useRandomPositions = true;
    [SerializeField] private float minDistance = 2f;  // Min distance between start/end
    [SerializeField] private float maxReach = 4f;     // Robot's max reach
    [SerializeField] private float workspaceRadius = 3.5f;  // Safe reachable area

    [Header("Curriculum Learning")]
    [Tooltip("After this many episodes, enable Power Budget and Random Positions")]
    [SerializeField] private int curriculumEpisodeThreshold = 500;
    private bool curriculumActive = false;

    [Header("Safety Zones")]
    [Tooltip("Penalty applied per frame when a joint is pushed against its physical limit")]
    [SerializeField] private float jointLimitPenalty = -0.001f;

    [Header("Action Smoothing")]
    [Range(0.01f, 1f)]
    [Tooltip("Lower = smoother/heavier movement, Higher = snappier/instant movement")]
    [SerializeField] private float actionSmoothing = 0.15f;

    [Header("Physics Data Collection")]
    [SerializeField] private bool collectDetailedPhysics = true;

    // Magnetic pickup system
    private bool isBoxAttached = false;
    private FixedJoint magnetJoint;
    private Vector3 boxStartPosition;
    private Vector3 targetPosition;

    // Performance tracking
    private float episodeStartTime;
    private float totalEnergyConsumed;
    private int successfulMoves;
    private int totalAttempts;
    private float distanceToTarget;
    private float previousDistanceToTarget;

    // Physics data for mechanics analysis
    private PhysicsData currentPhysicsData;
    private DataCollector dataCollector;

    // Inverse kinematics data
    private Vector2 currentJointAngles;  // [shoulder, elbow]
    private Vector3 previousMagnetPosition;
    private Vector3 magnetVelocity;

    public override void Initialize()
    {
        dataCollector = GetComponent<DataCollector>();
        if (dataCollector == null)
        {
            dataCollector = gameObject.AddComponent<DataCollector>();
        }

        boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
        targetPosition = targetZoneB.position + Vector3.up * 0.5f;
        successfulMoves = 0;
        totalAttempts = 0;
        SetupMagnetCollider();
        previousMagnetPosition = magnet.position;
    }

    private void SetupMagnetCollider()
    {
        SphereCollider magnetCollider = magnet.GetComponent<SphereCollider>();
        if (magnetCollider == null)
        {
            magnetCollider = magnet.gameObject.AddComponent<SphereCollider>();
        }
        magnetCollider.radius = magneticRange;
        magnetCollider.isTrigger = true;

        MagnetTrigger trigger = magnet.GetComponent<MagnetTrigger>();
        if (trigger == null)
        {
            trigger = magnet.gameObject.AddComponent<MagnetTrigger>();
        }
        trigger.Initialize(this);
    }

    public override void OnEpisodeBegin()
    {
        // Curriculum phase detection
        if (CompletedEpisodes >= curriculumEpisodeThreshold && !curriculumActive)
        {
            curriculumActive = true;
            usePowerBudget = true;
            useRandomPositions = true;
            Debug.Log("<color=green>Curriculum Phase 2: *Power Budget* and *Random Positions* Enabled!</color>");
        }

        DetachBox();
        ResetRobotArm();

        // Reset power budget
        if (usePowerBudget) currentPower = maxPowerBudget;

        // Generate random positions or use fixed zones
        // All positions should be relative to this training area's transform
        Vector3 areaOrigin = transform.position;
        
        if (useRandomPositions)
        {
            GenerateRandomPositions();
        }
        else
        {
            // Use the target zones that are children of this training area (already in correct world position)
            boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
            targetPosition = targetZoneB.position + Vector3.up * 0.5f;
        }

        if (movableBox != null)
        {
            movableBox.velocity = Vector3.zero;
            movableBox.angularVelocity = Vector3.zero;
            movableBox.transform.position = boxStartPosition;
            movableBox.transform.rotation = Quaternion.identity;
        }

        episodeStartTime = Time.time;
        totalEnergyConsumed = 0f;
        totalAttempts++;
        previousDistanceToTarget = Vector3.Distance(movableBox.position, targetPosition);
        previousMagnetPosition = magnet.position;

        if (collectDetailedPhysics)
        {
            currentPhysicsData = new PhysicsData();
            currentPhysicsData.episodeNumber = CompletedEpisodes + 1;
            currentPhysicsData.startTime = episodeStartTime;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Position data (9)
        sensor.AddObservation(transform.InverseTransformPoint(magnet.position));
        sensor.AddObservation(transform.InverseTransformPoint(movableBox.position));
        sensor.AddObservation(transform.InverseTransformPoint(targetPosition));

        // Distance data (3)
        float distanceToBox = Vector3.Distance(magnet.position, movableBox.position);
        sensor.AddObservation(distanceToBox);
        distanceToTarget = Vector3.Distance(movableBox.position, targetPosition);
        sensor.AddObservation(distanceToTarget);
        // Use local floor height relative to this training area
        float localFloorY = floor != null ? floor.position.y : transform.position.y;
        sensor.AddObservation(magnet.position.y - localFloorY);

        // Joint configuration (6)
        float baseAngle = GetJointAngle(baseRotation);
        float shoulderAngle = GetJointAngle(shoulderJoint);
        float elbowAngle = GetJointAngle(elbowJoint);
        sensor.AddObservation(baseAngle);
        sensor.AddObservation(shoulderAngle);
        sensor.AddObservation(elbowAngle);
        currentJointAngles = new Vector2(shoulderAngle, elbowAngle);
        sensor.AddObservation(baseRotation != null ? baseRotation.velocity[0] : 0f);
        sensor.AddObservation(shoulderJoint != null ? shoulderJoint.velocity[0] : 0f);
        sensor.AddObservation(elbowJoint != null ? elbowJoint.velocity[0] : 0f);

        // Velocity data (6)
        sensor.AddObservation(movableBox.velocity);
        magnetVelocity = (magnet.position - previousMagnetPosition) / Time.fixedDeltaTime;
        sensor.AddObservation(magnetVelocity);
        previousMagnetPosition = magnet.position;

        // State flags (7)
        sensor.AddObservation(isBoxAttached ? 1f : 0f);
        sensor.AddObservation(distanceToBox < magneticRange ? 1f : 0f);
        sensor.AddObservation(Vector3.Distance(movableBox.position, targetPosition) < 0.5f ? 1f : 0f);
        float timeElapsed = Time.time - episodeStartTime;
        sensor.AddObservation(Mathf.Clamp01(timeElapsed / 60f));
        // Use local box height relative to this training area's floor
        sensor.AddObservation(movableBox.position.y - localFloorY);
        float improvementRate = (previousDistanceToTarget - distanceToTarget) / Time.fixedDeltaTime;
        sensor.AddObservation(improvementRate);
        sensor.AddObservation(usePowerBudget ? currentPower / maxPowerBudget : 1f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float baseControl = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float shoulderControl = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float elbowControl = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        ApplyJointTorque(baseRotation, baseControl);
        ApplyJointTorque(shoulderJoint, shoulderControl);
        ApplyJointTorque(elbowJoint, elbowControl);

        float energyThisStep = CalculateEnergyConsumption(baseControl, shoulderControl, elbowControl);
        totalEnergyConsumed += energyThisStep;

        // Power budget check
        if (usePowerBudget)
        {
            currentPower -= energyThisStep;
            if (currentPower <= 0f)
            {
                AddReward(-5f);
                if (dataCollector != null)
                {
                    CollectEpisodeData(false);
                }
                EndEpisode();
                return;
            }
        }

        if (collectDetailedPhysics && currentPhysicsData != null)
        {
            CollectPhysicsSnapshot(baseControl, shoulderControl, elbowControl, energyThisStep);
        }

        CalculateRewards();
        CheckEpisodeEnd();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetKey(KeyCode.Q) ? -1f : Input.GetKey(KeyCode.E) ? 1f : 0f;
        continuousActions[1] = Input.GetKey(KeyCode.W) ? 1f : Input.GetKey(KeyCode.S) ? -1f : 0f;
        continuousActions[2] = Input.GetKey(KeyCode.A) ? -1f : Input.GetKey(KeyCode.D) ? 1f : 0f;
    }

    private float CalculateEnergyConsumption(float baseControl, float shoulderControl, float elbowControl)
    {
        float baseEnergy = Mathf.Abs(baseControl * (baseRotation != null ? baseRotation.velocity[0] : 0f));
        float shoulderEnergy = Mathf.Abs(shoulderControl * (shoulderJoint != null ? shoulderJoint.velocity[0] : 0f));
        float elbowEnergy = Mathf.Abs(elbowControl * (elbowJoint != null ? elbowJoint.velocity[0] : 0f));
        return (baseEnergy + shoulderEnergy + elbowEnergy) * Time.fixedDeltaTime;
    }

    private void CollectPhysicsSnapshot(float baseControl, float shoulderControl, float elbowControl, float energy)
    {
        PhysicsSnapshot snapshot = new PhysicsSnapshot
        {
            timestamp = Time.time - episodeStartTime,
            baseAngle = GetJointAngle(baseRotation),
            shoulderAngle = GetJointAngle(shoulderJoint),
            elbowAngle = GetJointAngle(elbowJoint),
            baseVelocity = baseRotation != null ? baseRotation.velocity[0] : 0f,
            shoulderVelocity = shoulderJoint != null ? shoulderJoint.velocity[0] : 0f,
            elbowVelocity = elbowJoint != null ? elbowJoint.velocity[0] : 0f,
            baseControl = baseControl,
            shoulderControl = shoulderControl,
            elbowControl = elbowControl,
            magnetPosition = magnet.position,
            magnetVelocity = magnetVelocity,
            boxPosition = movableBox.position,
            boxVelocity = movableBox.velocity,
            isBoxAttached = isBoxAttached,
            energyConsumed = energy
        };

        currentPhysicsData.snapshots.Add(snapshot);
    }

    private void CalculateRewards()
    {
        // Distance improvement reward
        float distanceImprovement = previousDistanceToTarget - distanceToTarget;
        AddReward(distanceImprovement * 2f * rewardMultiplier);
        previousDistanceToTarget = distanceToTarget;

        float distanceToBox = Vector3.Distance(magnet.position, movableBox.position);
        
        if (!isBoxAttached)
        {
            if (distanceToBox < 1f)
            {
                AddReward(0.05f * (1f - distanceToBox) * rewardMultiplier);
            }
            if (distanceToBox < magneticRange)
            {
                AddReward(0.1f * rewardMultiplier);
            }
        }
        else
        {
            AddReward(0.02f * rewardMultiplier);
            if (distanceImprovement > 0)
            {
                AddReward(0.5f * rewardMultiplier);
            }
            
            // Bonus for smooth, efficient movement while holding box
            if (magnetVelocity.magnitude < 1.5f)  // Moving smoothly
            {
                AddReward(0.01f * rewardMultiplier);
            }
        }

        // Energy penalties
        AddReward(-0.0005f * totalEnergyConsumed);
        AddReward(-0.0001f);  // Time penalty

        // Power efficiency bonus
        if (usePowerBudget)
        {
            float powerEfficiency = currentPower / maxPowerBudget;
            if (powerEfficiency > 0.5f)  // Still have >50% power
            {
                AddReward(0.005f * powerEfficiency);
            }
        }

        if (isBoxAttached)
        {
            float excessiveMovement = magnetVelocity.magnitude;
            if (excessiveMovement > 2f)
            {
                AddReward(-0.001f * excessiveMovement);
            }
        }

        // Success!
        if (distanceToTarget < 0.5f)
        {
            float baseReward = 20f;
            
            // Bonus for completing with power remaining
            if (usePowerBudget)
            {
                float powerBonus = (currentPower / maxPowerBudget) * 10f;
                baseReward += powerBonus;
            }
            
            // Bonus for completing quickly
            float timeTaken = Time.time - episodeStartTime;
            if (timeTaken < 30f)  // Under 30 seconds
            {
                float timeBonus = (30f - timeTaken) / 30f * 5f;
                baseReward += timeBonus;
            }
            
            AddReward(baseReward * rewardMultiplier);
            successfulMoves++;
            
            if (dataCollector != null)
            {
                CollectEpisodeData(true);
            }
            
            EndEpisode();
        }
    }

    private void CheckEpisodeEnd()
    {
        // Get local floor height for this training area
        float floorY = floor != null ? floor.position.y : transform.position.y;
        
        // Box fell through/off floor
        if (movableBox.position.y < floorY - 1f)
        {
            AddReward(-10f);
            if (dataCollector != null)
            {
                CollectEpisodeData(false);
            }
            EndEpisode();
            return;
        }

        // Max time exceeded
        if (Time.time - episodeStartTime > 60f)
        {
            AddReward(-3f);
            if (dataCollector != null)
            {
                CollectEpisodeData(false);
            }
            EndEpisode();
            return;
        }

        // Magnet went out of bounds - use LOCAL position relative to training area
        Vector3 localMagnetPos = magnet.position - transform.position;
        if (magnet.position.y < floorY - 2f || localMagnetPos.magnitude > 20f)
        {
            AddReward(-5f);
            EndEpisode();
        }
    }

    private void CollectEpisodeData(bool success)
    {
        float timeTaken = Time.time - episodeStartTime;
        float accuracy = 1f - (distanceToTarget / Vector3.Distance(boxStartPosition, targetPosition));
        accuracy = Mathf.Clamp01(accuracy);

        // Add physics data to current episode data
        if (collectDetailedPhysics && currentPhysicsData != null)
        {
            currentPhysicsData.timeTaken = timeTaken;
            currentPhysicsData.success = success;
            currentPhysicsData.finalAccuracy = accuracy;
            currentPhysicsData.totalEnergyConsumed = totalEnergyConsumed;
            currentPhysicsData.endTime = Time.time;
        }

        // Send to data collector
        dataCollector.RecordEpisode(timeTaken, accuracy, totalEnergyConsumed, success, currentPhysicsData);
        
        // NEW: Report to performance tracker
        if (PerformanceTracker.Instance != null)
        {
            PerformanceTracker.Instance.RecordEpisode(this, success, timeTaken, totalEnergyConsumed, accuracy);
        }
    }

    private void ApplyJointTorque(ArticulationBody joint, float control)
    {
        if (joint == null) return;

        var drive = joint.xDrive;
        
        // Apply action smoothing
        float desiredTarget = drive.target + control * movementSpeed * Time.fixedDeltaTime;
        drive.target = Mathf.Lerp(drive.target, desiredTarget, actionSmoothing);

        // Joint limit penalty (safety zones)
        if (drive.target <= drive.lowerLimit + 1f || drive.target >= drive.upperLimit - 1f)
        {
            AddReward(jointLimitPenalty);
        }

        drive.target = Mathf.Clamp(drive.target, drive.lowerLimit, drive.upperLimit);
        joint.xDrive = drive;
    }

    private float GetJointAngle(ArticulationBody joint)
    {
        if (joint == null) return 0f;
        return joint.xDrive.target;
    }

    private void GenerateRandomPositions()
    {
        // Circular workspace randomization for box start and target positions
        // Works around the robot base within defined radius
        // CONSTRAINT: Start and end must be at least 100 degrees apart (both CW and CCW)
        //             and at the SAME radius from the center
        // IMPORTANT: Positions are LOCAL to the training area, then converted to world space
        Vector3 areaOrigin = transform.position;  // This agent's training area origin
        
        // Use the same radius for both start and end positions
        float sharedRadius = Random.Range(1.5f, workspaceRadius);
        
        // Generate start angle randomly
        float startAngleDeg = Random.Range(0f, 360f);
        
        // Generate end angle that is at least 100 degrees away in BOTH directions
        // This means the end angle must be between 100 and 260 degrees away from start
        // (100 to 260 ensures at least 100 deg CW and at least 100 deg CCW)
        float minSeparation = 100f;
        float maxSeparation = 360f - minSeparation; // 260 degrees
        float angleSeparation = Random.Range(minSeparation, maxSeparation);
        
        // Randomly choose direction (CW or CCW)
        if (Random.value > 0.5f)
            angleSeparation = -angleSeparation;
        
        float endAngleDeg = startAngleDeg + angleSeparation;
        
        // Convert to radians
        float startAngle = startAngleDeg * Mathf.Deg2Rad;
        float endAngle = endAngleDeg * Mathf.Deg2Rad;
        
        Vector3 localStartPos = new Vector3(Mathf.Cos(startAngle) * sharedRadius, 0.75f, Mathf.Sin(startAngle) * sharedRadius);
        boxStartPosition = areaOrigin + localStartPos;

        Vector3 localEndPos = new Vector3(Mathf.Cos(endAngle) * sharedRadius, 0.75f, Mathf.Sin(endAngle) * sharedRadius);
        targetPosition = areaOrigin + localEndPos;

        if (targetZoneA) targetZoneA.position = new Vector3(boxStartPosition.x, areaOrigin.y + 0.05f, boxStartPosition.z);
        if (targetZoneB) targetZoneB.position = new Vector3(targetPosition.x, areaOrigin.y + 0.05f, targetPosition.z);
    }

    private void ResetRobotArm()
    {
        if (baseRotation != null)
        {
            var drive = baseRotation.xDrive;
            drive.target = 0f;
            baseRotation.xDrive = drive;
            baseRotation.jointVelocity = new ArticulationReducedSpace(0f);
        }

        if (shoulderJoint != null)
        {
            var drive = shoulderJoint.xDrive;
            drive.target = 45f;
            shoulderJoint.xDrive = drive;
            shoulderJoint.jointVelocity = new ArticulationReducedSpace(0f);
        }

        if (elbowJoint != null)
        {
            var drive = elbowJoint.xDrive;
            drive.target = -30f;
            elbowJoint.xDrive = drive;
            elbowJoint.jointVelocity = new ArticulationReducedSpace(0f);
        }
    }

    public void OnMagnetTriggerEnter(Collider other)
    {
        if (other.attachedRigidbody == movableBox && !isBoxAttached)
        {
            AttachBox();
        }
    }

    private void AttachBox()
    {
        if (magnetJoint != null) return;

        magnetJoint = magnet.gameObject.AddComponent<FixedJoint>();
        magnetJoint.connectedBody = movableBox;
        magnetJoint.enableCollision = false;
        magnetJoint.breakForce = magneticStrength;
        magnetJoint.breakTorque = magneticStrength;

        isBoxAttached = true;
        AddReward(1f * rewardMultiplier);
        Debug.Log($"[RobotAgent] Box attached! Episode {CompletedEpisodes}");
    }

    private void DetachBox()
    {
        if (magnetJoint != null)
        {
            Destroy(magnetJoint);
            magnetJoint = null;
        }
        isBoxAttached = false;
    }

    void OnDrawGizmos()
    {
        if (!visualizeMagnetRange || magnet == null) return;

        Gizmos.color = isBoxAttached ? Color.green : Color.yellow;
        Gizmos.DrawWireSphere(magnet.position, magneticRange);

        if (movableBox != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(magnet.position, movableBox.position);
        }
    }

    void OnGUI()
    {
        if (!Application.isPlaying) return;

        GUILayout.BeginArea(new Rect(10, 10, 350, 250));
        GUILayout.Label($"=== Robot Status ===");
        GUILayout.Label($"Episode: {CompletedEpisodes}");
        GUILayout.Label($"Current Curriculum: {(curriculumActive ? "Phase 2 (Hard)" : "Phase 1 (Easy)")}");
        GUILayout.Label($"Success Rate: {GetSuccessRate():F2}%");
        GUILayout.Label($"Box Attached: {(isBoxAttached ? "YES" : "NO")}");
        GUILayout.Label($"Distance to Target: {distanceToTarget:F2}m");
        GUILayout.Label($"Energy Used: {totalEnergyConsumed:F2}");
        if (usePowerBudget) GUILayout.Label($"Power: {currentPower / maxPowerBudget * 100f:F1}%");
        GUILayout.Label($"Action Smoothing: {actionSmoothing:F2}");
        GUILayout.Label($"");
        GUILayout.Label($"Joint Angles:");
        GUILayout.Label($"  Base: {GetJointAngle(baseRotation):F1}°");
        GUILayout.Label($"  Shoulder: {GetJointAngle(shoulderJoint):F1}°");
        GUILayout.Label($"  Elbow: {GetJointAngle(elbowJoint):F1}°");
        GUILayout.EndArea();
    }

    private float GetSuccessRate()
    {
        if (totalAttempts == 0) return 0f;
        return (successfulMoves / (float)totalAttempts) * 100f;
    }
}

public class MagnetTrigger : MonoBehaviour
{
    private RobotAgent agent;

    public void Initialize(RobotAgent parentAgent)
    {
        agent = parentAgent;
    }

    void OnTriggerEnter(Collider other)
    {
        if (agent != null)
        {
            agent.OnMagnetTriggerEnter(other);
        }
    }
}