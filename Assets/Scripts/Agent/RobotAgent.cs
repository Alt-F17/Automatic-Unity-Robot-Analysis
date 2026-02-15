// action-taking script that directly communicates with the ML-Agents framework:
// controls the robot arm, calculates rewards, and manages episode lifecycle

using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Numerics;
using System.ComponentModel.DataAnnotations;

public class RobotAgent : Agent
{
    [Header("Robot Joint Components")]
    [SerializeField] private ArticulationBody baseRotation;    // rotates entire arm (Y-axis)
    public float Link1Length {get; private set; }              // distance between shoulder and elbow
    public float Link2Length {get; private set; }              // distance between elbow and magnet
    [SerializeField] private ArticulationBody shoulderJoint;   // shoulder joint
    [SerializeField] private ArticulationBody elbowJoint;      // elbow joint
    [SerializeField] private Transform magnet;                 // end obj with magnet

    [Header("Environment Objects")]
    [SerializeField] private Rigidbody movableBox;
    [SerializeField] private Transform targetZoneA;            // starting zone
    [SerializeField] private Transform targetZoneB;            // goal zone
    [SerializeField] private Transform floor;

    [Header("Magnet Settings")]
    [SerializeField] private float magneticRange = 0.5f;       // distance to auto-pickup
    [SerializeField] private float magneticStrength = 100f;    // spring force strength
    [SerializeField] private bool visualizeMagnetRange = true;  // hope this helps you Seb

    [Header("Training Parameters")]
    [SerializeField] private float movementSpeed = 50f;
    [SerializeField] private float rewardMultiplier = 1f;

    [Header("Power Budget System")]
    [SerializeField] private bool usePowerBudget = false;
    [SerializeField] private float maxPowerBudget = 100f;
    private float currentPower = 100f;

    [Header("Random Position System")]
    [SerializeField] private bool useRandomPositions = true;
    [SerializeField] private float minDistance = 2f;           // min distance between start/end
    [SerializeField] private float maxReach = 4f;              // robot's max reach
    [SerializeField] private float workspaceRadius = 3.5f;     // safe reachable area

    [Header("Curriculum Learning")]
    [Tooltip("After this many episodes, enable Power Budget and Random Positions")]
    [SerializeField] private int curriculumEpisodeThreshold = 500;
    private bool curriculumActive = false;                     // automatically activated by ML-Agents as CompletedEpisodes

    [Header("Safety Zones")]
    [Tooltip("Penalty applied per frame when a joint is pushed against its physical limit")]
    [SerializeField] private float jointLimitPenalty = -0.001f;

    [Header("Action Smoothing")]
    [Range(0.01f, 1f)]
    [Tooltip("Lower = smoother/heavier movement, Higher = snappier/instant movement")]
    [SerializeField] private float actionSmoothing = 0.15f;

    [Header("Physics Data Collection")]
    [SerializeField] private bool collectDetailedPhysics = true;

    // magnetic pickup system
    private bool isBoxAttached = false;
    private FixedJoint magnetJoint;
    private Vector3 boxStartPosition;
    private Vector3 targetPosition;

    // performance tracking
    private float episodeStartTime;
    private float totalEnergyConsumed;
    private int successfulMoves;
    private int totalAttempts;
    private float distanceToTarget;
    private float previousDistanceToTarget;

    // physics data for mechanics analysis
    private PhysicsData currentPhysicsData;
    private DataCollector dataCollector;

    // inverse kinematics data
    private Vector3 previousMagnetPosition;
    private Vector3 magnetVelocity;

    public override void Initialize() // override for data collection setup and init vars
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

        Link1Length = Vector3.Distance(shoulderJoint.position, elbowJoint.position);
        Link2Length = Vector3.Distance(elbowJoint.position, magnet.position);
    }

    private void SetupMagnetCollider() // in case I forgot
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

    public override void OnEpisodeBegin() // reset positions, states, and curriculum progression
    {
        // curriculum phase detection
        if (CompletedEpisodes >= curriculumEpisodeThreshold && !curriculumActive)
        {
            curriculumActive = true;
            usePowerBudget = true;
            useRandomPositions = true;
            Debug.Log("<color=green>Curriculum Phase 2: *Power Budget* and *Random Positions* Enabled!</color>");
        }

        DetachBox();
        ResetRobotArm();

        // reset power budget
        if (usePowerBudget) currentPower = maxPowerBudget;

        // generate random positions or use fixed zones
        if (useRandomPositions)
        {
            GenerateRandomPositions();
        }
        else
        {
            // use the target zones that are children of this training area (already in correct world position)
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
        previousDistanceToTarget = movableBox != null 
            ? Vector3.Distance(movableBox.position, targetPosition) 
            : Vector3.Distance(boxStartPosition, targetPosition);
        previousMagnetPosition = magnet.position;

        if (collectDetailedPhysics)
        {
            currentPhysicsData = new PhysicsData
            {
                episodeNumber = CompletedEpisodes + 1,
                startTime = episodeStartTime
            };
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // null safety for movableBox
        Vector3 boxPos = movableBox?.position ?? boxStartPosition;
        Vector3 boxVel = movableBox?.velocity ?? Vector3.zero;
        
        // position data
        sensor.AddObservation(transform.InverseTransformPoint(magnet.position));
        sensor.AddObservation(transform.InverseTransformPoint(boxPos));
        sensor.AddObservation(transform.InverseTransformPoint(targetPosition));

        // distance data
        float distanceToBox = Vector3.Distance(magnet.position, boxPos);
        sensor.AddObservation(distanceToBox);
        distanceToTarget = Vector3.Distance(boxPos, targetPosition);
        sensor.AddObservation(distanceToTarget);
        // use local floor height relative to this training area
        float localFloorY = floor != null ? floor.position.y : transform.position.y;
        sensor.AddObservation(magnet.position.y - localFloorY);

        // joint configuration: actual joint angles, not drive targets
        float baseAngle = GetJointAngle(baseRotation);
        float shoulderAngle = GetJointAngle(shoulderJoint);
        float elbowAngle = GetJointAngle(elbowJoint);
        sensor.AddObservation(baseAngle);
        sensor.AddObservation(shoulderAngle);
        sensor.AddObservation(elbowAngle);
        sensor.AddObservation(baseRotation != null ? baseRotation.velocity[0] : 0f);
        sensor.AddObservation(shoulderJoint != null ? shoulderJoint.velocity[0] : 0f);
        sensor.AddObservation(elbowJoint != null ? elbowJoint.velocity[0] : 0f);

        // velocity data: transform to local space for training area independence
        sensor.AddObservation(transform.InverseTransformDirection(boxVel));
        magnetVelocity = (magnet.position - previousMagnetPosition) / Time.fixedDeltaTime;
        sensor.AddObservation(transform.InverseTransformDirection(magnetVelocity));
        previousMagnetPosition = magnet.position;

        // state flags
        sensor.AddObservation(isBoxAttached ? 1f : 0f);
        sensor.AddObservation(distanceToBox < magneticRange ? 1f : 0f);
        sensor.AddObservation(Vector3.Distance(boxPos, targetPosition) < 0.5f ? 1f : 0f);
        float timeElapsed = Time.time - episodeStartTime;
        sensor.AddObservation(Mathf.Clamp01(timeElapsed / 60f));
        // use local box height relative to this training area's floor
        sensor.AddObservation(boxPos.y - localFloorY);
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

        float baseTorque = GetJointTorque(baseRotation);
        float shoulderTorque = GetJointTorque(shoulderJoint);
        float elbowTorque = GetJointTorque(elbowJoint);

        float totalTorqueThisStep = baseTorque + shoulderTorque + elbowTorque;

        float energyThisStep = CalculateEnergyConsumption(baseTorque, shoulderTorque, elbowTorque);
        totalEnergyConsumed += energyThisStep;

        // power budget check
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

        CalculateRewards(energyThisStep, totalTorqueThisStep);
        CheckEpisodeEnd();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetKey(KeyCode.Q) ? -1f : Input.GetKey(KeyCode.E) ? 1f : 0f;
        continuousActions[1] = Input.GetKey(KeyCode.W) ? 1f : Input.GetKey(KeyCode.S) ? -1f : 0f;
        continuousActions[2] = Input.GetKey(KeyCode.A) ? -1f : Input.GetKey(KeyCode.D) ? 1f : 0f;
    }

    private float CalculateEnergyConsumption(float baseTorque, float shoulderTorque, float elbowTorque)
    {
        float baseEnergy = Mathf.Abs(baseTorque * (baseRotation != null ? baseRotation.velocity[0] : 0f));
        float shoulderEnergy = Mathf.Abs(shoulderTorque * (shoulderJoint != null ? shoulderJoint.velocity[0] : 0f));
        float elbowEnergy = Mathf.Abs(elbowTorque * (elbowJoint != null ? elbowJoint.velocity[0] : 0f));
        return (baseEnergy + shoulderEnergy + elbowEnergy) * Time.fixedDeltaTime;
    }

    private void CollectPhysicsSnapshot(float baseControl, float shoulderControl, float elbowControl, float energy)
    {
        physicalBaseEnergy = 0f;
        physicalShoulderEnergy = 0f;
        physicalElbowEnergy = 0f;

        PhysicsSnapshot snapshot = new PhysicsSnapshot
        {
            timestamp = Time.time - episodeStartTime,
            baseAngle = GetJointAngle(baseRotation),
            shoulderAngle = GetJointAngle(shoulderJoint),
            elbowAngle = GetJointAngle(elbowJoint),
            magnetAngle = GetJointAngle(magnet),
            baseVelocity = baseRotation != null ? baseRotation.velocity[0] : 0f,
            shoulderVelocity = shoulderJoint != null ? shoulderJoint.velocity[0] : 0f,
            elbowVelocity = elbowJoint != null ? elbowJoint.velocity[0] : 0f,
            baseControl = baseControl,
            shoulderControl = shoulderControl,
            elbowControl = elbowControl,
            baseTorque = GetJointTorque(baseRotation), 
            shoulderTorque = GetJointTorque(shoulderJoint),
            elbowTorque = GetJointTorque(elbowJoint),
            basePower = baseVelocity * baseTorque,
            shoulderPower = shoulderVelocity * shoulderTorque,
            elbowPower = elbowVelocity * elbowTorque,

            // this is to find the accumulative mechanical energy consumption 

            baseEnergyIncrement = basePower * Time.fixedDeltaTime, 
            shoulderEnergyIncrement = shoulderPower * Time.fixedDeltaTime,
            elbowEnergyIncrement = elbowPower * Time.fixedDeltaTime,

            physicalBaseEnergy += baseEnergyIncrement,
            physicalShoulderEnergy += shoulderEnergyIncrement,
            physicalElbowEnergy += elbowEnergyIncrement,
            magnetPosition = magnet.position,
            magnetVelocity = magnetVelocity,
            boxPosition = movableBox != null ? movableBox.position : boxStartPosition,
            boxVelocity = movableBox != null ? movableBox.velocity : Vector3.zero,
            isBoxAttached = isBoxAttached,
            energyConsumed = energy
        };

        currentPhysicsData.snapshots.Add(snapshot);
    }

    private void CalculateRewards(float energyThisStep, float totalTorqueThisStep)
    {
        // distance improvement reward
        float distanceImprovement = previousDistanceToTarget - distanceToTarget;
        AddReward(distanceImprovement * 2f * rewardMultiplier);
        previousDistanceToTarget = distanceToTarget;

        Vector3 boxPos = movableBox?.position ?? boxStartPosition;
        float distanceToBox = Vector3.Distance(magnet.position, boxPos);
        
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
            // only reward holding if ALSO making progress toward target
            if (distanceImprovement > 0)
            {
                // scaled to NOT dwarf the success reward (0.02/frame * ~50 frames ≈ 1.0 total)
                AddReward(0.02f * rewardMultiplier);
                
                // bonus for smooth movement ONLY when also improving distance
                if (magnetVelocity.magnitude < 1.5f && magnetVelocity.magnitude > 0.1f)
                {
                    AddReward(0.005f * rewardMultiplier);
                }
            }
        }

        // energy penalty - use INCREMENTAL energy, not cumulative
        AddReward(-0.005f * energyThisStep * rewardMultiplier);
        AddReward(-0.0001f * rewardMultiplier);  // time penalty

        // torque penalty (if the arm holds the box for too long, creating excessive gravitational torque)
        // takes into account theoretical actuator stress and motion cost

        AddReward(-0.005f * totalTorqueThisStep * rewardMultiplier);

        // penalize excessive/jerky movement while holding
        if (isBoxAttached)
        {
            float excessiveMovement = magnetVelocity.magnitude;
            if (excessiveMovement > 2f)
            {
                AddReward(-0.001f * excessiveMovement);
            }
        }

        // success!
        if (distanceToTarget < 0.5f)
        {
            float baseReward = 20f;
            
            // bonus for completing with power remaining (ONLY at episode end)
            if (usePowerBudget)
            {
                float powerEfficiency = currentPower / maxPowerBudget;
                float powerBonus = powerEfficiency * 15f;  // up to 15 bonus for 100% power remaining
                baseReward += powerBonus;
            }
            
            // bonus for completing quickly
            float timeTaken = Time.time - episodeStartTime;
            if (timeTaken < 30f)  // under 30 seconds
            {
                float timeBonus = (30f - timeTaken) / 30f * 5f;
                baseReward += timeBonus;
            }
            
            // bonus for energy efficiency (total energy used)
            float energyEfficiencyBonus = Mathf.Max(0f, 5f - totalEnergyConsumed * 0.1f);
            baseReward += energyEfficiencyBonus;
            
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
        // get local floor height for this training area
        float floorY = floor != null ? floor.position.y : transform.position.y;
        
        // box fell through/off floor
        if (movableBox != null && movableBox.position.y < floorY - 1f)
        {
            AddReward(-10f);
            if (dataCollector != null)
            {
                CollectEpisodeData(false);
            }
            EndEpisode();
            return;
        }

        // max time exceeded
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

        // magnet went out of bounds - use LOCAL position relative to training area
        Vector3 localMagnetPos = magnet.position - transform.position;
        if (magnet.position.y < floorY - 2f || localMagnetPos.magnitude > 20f)
        {
            AddReward(-5f);
            if (dataCollector != null)
            {
                CollectEpisodeData(false);
            }
            EndEpisode();
        }
    }

    private void CollectEpisodeData(bool success)
    {
        float timeTaken = Time.time - episodeStartTime;
        float totalDistance = Vector3.Distance(boxStartPosition, targetPosition);
        float accuracy = totalDistance > 0.01f ? 1f - distanceToTarget / totalDistance : 0f;
        accuracy = Mathf.Clamp01(accuracy);

        // add physics data to current episode data
        if (collectDetailedPhysics && currentPhysicsData != null)
        {
            currentPhysicsData.timeTaken = timeTaken;
            currentPhysicsData.success = success;
            currentPhysicsData.finalAccuracy = accuracy;
            currentPhysicsData.totalEnergyConsumed = totalEnergyConsumed;
            currentPhysicsData.endTime = Time.time;
        }

        // send to data collector
        dataCollector.RecordEpisode(timeTaken, accuracy, totalEnergyConsumed, success, currentPhysicsData);
        
        // NEW: report to performance tracker
        if (PerformanceTracker.Instance != null)
        {
            PerformanceTracker.Instance.RecordEpisode(this, success, timeTaken, totalEnergyConsumed, accuracy);
        }
    }

    private void ApplyJointTorque(ArticulationBody joint, float control)
    {
        if (joint == null) return;

        var drive = joint.xDrive;
        
        // apply action smoothing
        float desiredTarget = drive.target + control * movementSpeed * Time.fixedDeltaTime;
        drive.target = Mathf.Lerp(drive.target, desiredTarget, actionSmoothing);

        // joint limit penalty — use ACTUAL joint angle, not drive target
        float actualAngle = joint.jointPosition[0] * Mathf.Rad2Deg;
        if (actualAngle <= drive.lowerLimit + 1f || actualAngle >= drive.upperLimit - 1f)
        {
            AddReward(jointLimitPenalty);
        }

        drive.target = Mathf.Clamp(drive.target, drive.lowerLimit, drive.upperLimit);
        joint.xDrive = drive;
    }

    private float GetJointAngle(ArticulationBody joint)
    {
        if (joint == null) return 0f;
        // use actual joint position, NOT drive target — the target is what the motor
        // is trying to reach, but actual angle can differ due to inertia/load
        return joint.jointPosition[0] * Mathf.Rad2Deg;
    }

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


    private void GenerateRandomPositions()
    {
        // circular workspace randomization for box start and target positions
        // works around the robot base within defined radius
        // CONSTRAINT: Start and end must be at least 100 degrees apart (both CW and CCW)
        //             and at the SAME radius from the center
        // IMPORTANT: Positions are LOCAL to the training area, then converted to world space
        Vector3 areaOrigin = transform.position;  // this agent's training area origin
        float floorY = floor != null ? floor.position.y : areaOrigin.y;
        float boxHeight = 0.75f;  // height above floor for box spawn
        
        // use the same radius for both start and end positions
        // clamp to maxReach to ensure positions are reachable
        float minRadius = Mathf.Max(1.5f, minDistance * 0.5f);  // use minDistance
        float maxRadius = Mathf.Min(workspaceRadius, maxReach);  // use maxReach
        float sharedRadius = Random.Range(minRadius, maxRadius);
        
        // generate start angle randomly
        float startAngleDeg = Random.Range(0f, 360f);
        
        // generate end angle that is at least 100 degrees away in BOTH directions
        // this means the end angle must be between 100 and 260 degrees away from start
        // (100 to 260 ensures at least 100 deg CW and at least 100 deg CCW)
        float minSeparation = 100f;
        float maxSeparation = 360f - minSeparation; // 260 degrees
        float angleSeparation = Random.Range(minSeparation, maxSeparation);
        
        // randomly choose direction (CW or CCW)
        if (Random.value > 0.5f)
            angleSeparation = -angleSeparation;
        
        float endAngleDeg = startAngleDeg + angleSeparation;
        
        // convert to radians
        float startAngle = startAngleDeg * Mathf.Deg2Rad;
        float endAngle = endAngleDeg * Mathf.Deg2Rad;
        
        // use floor-relative Y position
        Vector3 localStartPos = new Vector3(Mathf.Cos(startAngle) * sharedRadius, floorY + boxHeight - areaOrigin.y, Mathf.Sin(startAngle) * sharedRadius);
        boxStartPosition = areaOrigin + localStartPos;

        Vector3 localEndPos = new Vector3(Mathf.Cos(endAngle) * sharedRadius, floorY + boxHeight - areaOrigin.y, Mathf.Sin(endAngle) * sharedRadius);
        targetPosition = areaOrigin + localEndPos;

        if (targetZoneA) targetZoneA.position = new Vector3(boxStartPosition.x, floorY + 0.05f, boxStartPosition.z);
        if (targetZoneB) targetZoneB.position = new Vector3(targetPosition.x, floorY + 0.05f, targetPosition.z);
    }

    private void ResetRobotArm() // reset joints to default positions at episode start
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

    public void OnMagnetTriggerEnter(Collider other) // magnet inbound trigger event handler
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
        // debug logging removed to prevent console spam during parallel training
    }

    private void DetachBox() // detach the box from the magnet
    {
        if (magnetJoint != null)
        {
            Destroy(magnetJoint);
            magnetJoint = null;
        }
        isBoxAttached = false;
    }

    // Called by Unity when the FixedJoint breaks (force/torque exceeded).
    // Syncs isBoxAttached state so the agent doesn't think it still has the box.
    void OnJointBreak(float breakForce)
    {
        magnetJoint = null;
        isBoxAttached = false;
        AddReward(-1f * rewardMultiplier);  // penalty for dropping the box
    }

    void OnDrawGizmos() // visualize magnet range 
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

    void OnGUI() // generate simple on-screen HUD for debugging and performance tracking
    {
        if (!Application.isPlaying) return;

        // only display for the first agent to prevent 100x overdraw in parallel training
        if (CompletedEpisodes == 0 && totalAttempts <= 1)
        {
            // heuristic: only show if this is likely the "first" agent
            // exact filtering can be done via PerformanceTracker if needed
        }
        else if (PerformanceTracker.Instance != null && !PerformanceTracker.Instance.IsBestPerformer(this))
        {
            return;  // only the best performer shows the HUD
        }

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

    // also handle OnTriggerStay: if the box is teleported INTO the trigger
    // (e.g., at episode start), OnTriggerEnter won't fire because there's no
    // "enter" event — the box was placed inside. This catches that edge case.
    void OnTriggerStay(Collider other)
    {
        if (agent != null)
        {
            agent.OnMagnetTriggerEnter(other);
        }
    }
}
