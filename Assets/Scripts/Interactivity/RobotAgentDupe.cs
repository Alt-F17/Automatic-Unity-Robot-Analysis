// action-taking script that directly communicates with the ML-Agents framework:
// controls the robot arm, calculates rewards, and manages episode lifecycle

using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RobotAgentDupe : Agent
{
    [Header("Robot Joint Components")]
    [SerializeField] private ArticulationBody rootBody;        // root ArticulationBody (Fixed) — Base
    [SerializeField] private ArticulationBody baseRotation;    // rotates entire arm (Y-axis) — Body
    [SerializeField] private ArticulationBody shoulderJoint;   // shoulder joint — UpperArm
    [SerializeField] private ArticulationBody elbowJoint;      // elbow joint — ForeArm
    [SerializeField] private Transform magnet;                 // end obj with magnet — Hand

    [Header("Environment Objects")]
    [SerializeField] private Rigidbody movableBox;
    [SerializeField] public Transform targetZoneA;            // starting zone
    [SerializeField] public Transform targetZoneB;            // goal zone
    [SerializeField] private Transform floor;

    [Header("Magnet Settings")]
    [SerializeField] private float magneticRange = 0.5f;       // distance to auto-pickup
    // magneticStrength removed — no longer using breakable FixedJoint; box attaches via kinematic parenting
    [SerializeField] private bool visualizeMagnetRange = true;  // hope this helps you Seb

    [Header("Training Parameters")]
    [SerializeField] private float movementSpeed = 50f;
    [SerializeField] private float rewardMultiplier = 1f;

    [Header("Auto-Scaling (for large arms)")]
    [Tooltip("Automatically scale motor forces based on arm dimensions")]
    [SerializeField] private bool autoScaleForces = true;
    [SerializeField] private float baseStiffness = 100000f;
    [SerializeField] private float baseDamping = 10000f;
    [SerializeField] private float baseForceLimit = 100000f;
    private float armScaleFactor = 1f;

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
    [SerializeField] private int curriculumEpisodeThreshold = 250;
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
    // No longer using FixedJoint — kinematic parenting avoids AB solver conflicts and break-force issues
    private bool boxParented = false;
    private Renderer boxRenderer;
    private Color boxDefaultColor;
    private bool episodeEnding = false;  // prevents re-triggering success while flash coroutine runs
    private float attachCooldown = 0f; // prevents auto-reattach loops
    private Vector3 boxStartPosition;
    private Vector3 targetPosition;

    // performance tracking
    private float episodeStartTime;
    private float totalEnergyConsumed;
    private int successfulMoves;
    private int totalAttempts;
    private float distanceToTarget;
    private float previousDistanceToTarget;

    // smooth control tracking
    private float prevBaseControl = 0f;
    private float prevShoulderControl = 0f;
    private float prevElbowControl = 0f;

    // physics data for mechanics analysis
    private PhysicsData currentPhysicsData;
    private DataCollector dataCollector;

    // inverse kinematics data
    private Vector3 previousMagnetPosition;
    private Vector3 magnetVelocity;

    // Physics layer indices — must match ProjectSettings/TagManager.asset
    private const int LayerRobotPart   = 8;  // "RobotPart"
    private const int LayerTrainingBox = 9;  // "TrainingBox"

    public override void Initialize() // override for data collection setup and init vars
    {
        dataCollector = GetComponent<DataCollector>();
        if (dataCollector == null)
        {
            dataCollector = gameObject.AddComponent<DataCollector>();
        }

        // Configure joint drives based on arm scale
        if (autoScaleForces)
        {
            ConfigureJointDrives();
        }

        boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
        targetPosition = targetZoneB.position + Vector3.up * 0.5f;
        successfulMoves = 0;
        totalAttempts = 0;
        SetupMagnetCollider();
        previousMagnetPosition = magnet.position;

        // cache box renderer for colour feedback
        if (movableBox != null)
        {
            boxRenderer = movableBox.GetComponent<Renderer>();
            if (boxRenderer != null) boxDefaultColor = boxRenderer.material.color;
        }

        SetupPhysicsLayers();
        SetupZoneBTrigger();
    }

    private void SetupPhysicsLayers()
    {
        // Globally block RobotPart <-> TrainingBox collisions across all 100 areas.
        // Each agent calls this; it is idempotent.
        Physics.IgnoreLayerCollision(LayerRobotPart, LayerTrainingBox, true);

        // Assign all arm colliders to RobotPart (recursive)
        if (rootBody != null)
        {
            SetLayerRecursively(rootBody.gameObject, LayerRobotPart);
        }

        // Assign own box to TrainingBox (recursive)
        if (movableBox != null)
            SetLayerRecursively(movableBox.gameObject, LayerTrainingBox);

        // Re-enable own arm <-> own box collision (overrides the layer matrix per-pair).
        // Do this for ALL colliders on each side to keep magnet trigger + box collider working.
        if (movableBox != null && rootBody != null)
        {
            Collider[] boxCols = movableBox.GetComponentsInChildren<Collider>(true);
            Collider[] armCols = rootBody.GetComponentsInChildren<Collider>(true);
            if (boxCols != null && armCols != null)
            {
                foreach (Collider boxCol in boxCols)
                {
                    foreach (Collider armCol in armCols)
                    {
                        Physics.IgnoreCollision(armCol, boxCol, false);
                    }
                }
            }
        }
    }

    private void SetLayerRecursively(GameObject root, int layer)
    {
        if (root == null) return;
        root.layer = layer;
        foreach (Transform child in root.transform)
        {
            SetLayerRecursively(child.gameObject, layer);
        }
    }

    private void SetupZoneBTrigger()
    {
        if (targetZoneB == null || movableBox == null) return;
        ZoneTrigger zt = targetZoneB.GetComponent<ZoneTrigger>();
        if (zt == null) zt = targetZoneB.gameObject.AddComponent<ZoneTrigger>();
        zt.Initialize(movableBox, OnBoxLandedOnZoneB);
    }

    /// Called by ZoneTrigger when this area's box physically enters zone B.
    public void OnBoxLandedOnZoneB()
    {
        if (episodeEnding || !isActiveAndEnabled) return;
        episodeEnding = true; // Lock immediately to prevent double-firing

        Debug.Log($"<color=green>[SUCCESS] Box landed on Zone B!</color>");

        if (isBoxAttached)
        {
            Debug.Log($"<color=yellow>[AUTO-DETACH] Released at target...</color>");
            DetachBox();
        }

        // Increased success reward to 60f to make it the ultimate priority
        float baseReward = 60f;

        if (usePowerBudget)
        {
            float powerEfficiency = currentPower / maxPowerBudget;
            baseReward += powerEfficiency * 15f;
        }

        float timeTaken = Time.time - episodeStartTime;
        if (timeTaken < 30f)
            baseReward += (30f - timeTaken) / 30f * 5f;

        float energyEfficiencyBonus = Mathf.Max(0f, 5f - totalEnergyConsumed * 0.1f);
        baseReward += energyEfficiencyBonus;

        AddReward(baseReward * rewardMultiplier);
        successfulMoves++;

        SetBoxColor(Color.green);

        if (dataCollector != null)
            CollectEpisodeData(true);

        StartCoroutine(SuccessFlashThenEnd());
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
        attachCooldown = 0f; // reset lock
        
        // reset smooth control trackers
        prevBaseControl = 0f;
        prevShoulderControl = 0f;
        prevElbowControl = 0f;

        // curriculum phase detection
        if (CompletedEpisodes >= curriculumEpisodeThreshold && !curriculumActive)
        {
            curriculumActive = true;
            usePowerBudget = true;
            useRandomPositions = true;
            Debug.Log("<color=green>Curriculum Phase 2: *Power Budget* and *Random Positions* Enabled!</color>");
        }

        DetachBox();
        episodeEnding = false;
        SetBoxColor(boxDefaultColor);  // clear any success green from last episode
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
            Debug.Log($"<color=cyan>[ZONES] Start={boxStartPosition}, Target={targetPosition}, Distance={Vector3.Distance(boxStartPosition, targetPosition):F2}m</color>");
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
            ? Vector3.Distance(movableBox.transform.position, targetPosition) 
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
        // null safety for movableBox, always read visual transform to avoid kinematic sleep physics bugs
        Vector3 boxPos = movableBox != null ? movableBox.transform.position : boxStartPosition;
        Vector3 boxVel = movableBox?.velocity ?? Vector3.zero;

        // position data
        sensor.AddObservation(transform.InverseTransformPoint(magnet.position));
        sensor.AddObservation(transform.InverseTransformPoint(boxPos));
        sensor.AddObservation(transform.InverseTransformPoint(targetPosition));

        // distance data
        float distanceToBox = Vector3.Distance(magnet.position, boxPos);
        sensor.AddObservation(distanceToBox);
        
        Vector3 boxPosXZ = new Vector3(boxPos.x, 0, boxPos.z);
        Vector3 targetPosXZ = new Vector3(targetPosition.x, 0, targetPosition.z);
        distanceToTarget = Vector3.Distance(boxPosXZ, targetPosXZ);
        sensor.AddObservation(distanceToTarget);
        // use local floor height relative to this training area
        float localFloorY = floor != null ? floor.position.y : transform.position.y;
        sensor.AddObservation(magnet.position.y - localFloorY);

        // joint configuration: actual joint angles, normalized so FreeMotion joints don't spin to infinity
        float baseAngle = GetNormalizedJointAngle(baseRotation);
        float shoulderAngle = GetNormalizedJointAngle(shoulderJoint);
        float elbowAngle = GetNormalizedJointAngle(elbowJoint);
        sensor.AddObservation(baseAngle);
        sensor.AddObservation(shoulderAngle);
        sensor.AddObservation(elbowAngle);
        sensor.AddObservation(GetJointVelocity(baseRotation));
        sensor.AddObservation(GetJointVelocity(shoulderJoint));
        sensor.AddObservation(GetJointVelocity(elbowJoint));

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
        if (episodeEnding) return; // Prevent brain from overriding success animation

        float baseControl = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float shoulderControl = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float elbowControl = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        ApplyJointTorque(baseRotation, baseControl);
        ApplyJointTorque(shoulderJoint, shoulderControl);
        ApplyJointTorque(elbowJoint, elbowControl);

        float energyThisStep = CalculateEnergyConsumption(baseControl, shoulderControl, elbowControl);
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

        // action jitter penalty (elegance/fluidity)
        float actionJitter = Mathf.Abs(baseControl - prevBaseControl) + 
                             Mathf.Abs(shoulderControl - prevShoulderControl) + 
                             Mathf.Abs(elbowControl - prevElbowControl);
        
        // heavily penalize rapid oscillating inputs (jittering)
        if (actionJitter > 0.5f) {
            AddReward(-0.002f * actionJitter * rewardMultiplier);
        }

        prevBaseControl = baseControl;
        prevShoulderControl = shoulderControl;
        prevElbowControl = elbowControl;

        CalculateRewards(energyThisStep);
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
        // Instead of relying on Unity's buggy ArticulationBody.jointForce (which often returns 0 
        // depending on drive types) or velocity (which would evaluate to 0 when holding a heavy box still),
        // we calculate effort based on the actual control impulses being sent by the neural network.
        // We multiply by fixedDeltaTime so it scales correctly with the time step.
        float baseEnergy = Mathf.Abs(baseControl);
        float shoulderEnergy = Mathf.Abs(shoulderControl);
        float elbowEnergy = Mathf.Abs(elbowControl);
        
        // Multiply by a factor (e.g., 10f) to tune how quickly the 100 power budget drains.
        // If max power is 100, and they use full 1.0 control on all 3 joints, 
        // they consume 3 * 10 * 0.02 = 0.6 power per frame, draining 100 power in ~166 frames (~3 seconds).
        // Let's use a scale factor of 2f so they have about ~20 seconds of continuous full-force movement.
        float powerDrainScale = 2f; 
        return (baseEnergy + shoulderEnergy + elbowEnergy) * powerDrainScale * Time.fixedDeltaTime;
    }

    private float GetGravitationalLoad(ArticulationBody joint, ArticulationBody childBody)
    {
        if (joint == null || childBody == null) return 0f;

        // F = m * g
        // We include the payload mass if the box is attached to the hand!
        float totalMass = childBody.mass;
        if (isBoxAttached && movableBox != null)
        {
            // Adding half the mass of the box recursively to the joints
            totalMass += movableBox.mass;
        }

        float force = totalMass * Mathf.Abs(Physics.gravity.y);
        
        // r = distance from the joint pivot to the child's center of mass
        Vector3 jointPivot = joint.transform.position;
        Vector3 outLever = childBody.transform.TransformPoint(childBody.centerOfMass) - jointPivot;
        
        // If box is attached and this is the elbow (closest to the box), the lever arm extends further to the magnet
        if (isBoxAttached && joint == elbowJoint && magnet != null)
        {
            outLever = magnet.position - jointPivot;
        }

        float radius = outLever.magnitude;

        // theta = angle between the lever arm and gravity (down)
        float angle = Vector3.Angle(outLever, Vector3.down);
        float sinTheta = Mathf.Sin(angle * Mathf.Deg2Rad);

        // Torque = F * r * sin(theta)
        return force * radius * sinTheta;
    }

    private void CollectPhysicsSnapshot(float baseControl, float shoulderControl, float elbowControl, float energy)
    {
        // calculate joint angular velocities using actual joint data (rad/s), not linear velocity
        float baseVelocity = GetJointVelocity(baseRotation);
        float shoulderVelocity = GetJointVelocity(shoulderJoint);
        float elbowVelocity = GetJointVelocity(elbowJoint);

        // exact torque from physics solver (jointForce returns the force/torque applied by the drive)
        float baseTorque = GetJointTorque(baseRotation);
        float shoulderTorque = GetJointTorque(shoulderJoint);
        float elbowTorque = GetJointTorque(elbowJoint);

        // Calculate theoretical external gravitational load (τ = F × r × sin(θ))
        // Base joint only rotates around Y (gravity doesn't pull it along its DOF), so 0
        float baseGrav = 0f; 
        float shoulderGrav = GetGravitationalLoad(baseRotation, shoulderJoint);
        float elbowGrav = GetGravitationalLoad(shoulderJoint, elbowJoint);

        PhysicsSnapshot snapshot = new PhysicsSnapshot
        {
            timestamp = Time.time - episodeStartTime,

            baseAngle = GetNormalizedJointAngle(baseRotation),
            shoulderAngle = GetNormalizedJointAngle(shoulderJoint),
            elbowAngle = GetNormalizedJointAngle(elbowJoint),

            baseVelocity = baseVelocity,
            shoulderVelocity = shoulderVelocity,
            elbowVelocity = elbowVelocity,

            baseControl = baseControl,
            shoulderControl = shoulderControl,
            elbowControl = elbowControl,

            baseTorque = baseTorque,
            shoulderTorque = shoulderTorque,
            elbowTorque = elbowTorque,
            
            baseGravTorque = baseGrav,
            shoulderGravTorque = shoulderGrav,
            elbowGravTorque = elbowGrav,

            basePower = baseVelocity * baseTorque,
            shoulderPower = shoulderVelocity * shoulderTorque,
            elbowPower = elbowVelocity * elbowTorque,

            // Actual masses from ArticulationBody (kg)
            baseMass = baseRotation != null ? baseRotation.mass : 0f,
            shoulderMass = shoulderJoint != null ? shoulderJoint.mass : 0f,
            elbowMass = elbowJoint != null ? elbowJoint.mass : 0f,

            // Rotational kinetic energy per joint: 0.5 * m * omega^2 (simplified)
            baseKineticEnergy = baseRotation != null ? 0.5f * baseRotation.mass * baseVelocity * baseVelocity : 0f,
            shoulderKineticEnergy = shoulderJoint != null ? 0.5f * shoulderJoint.mass * shoulderVelocity * shoulderVelocity : 0f,
            elbowKineticEnergy = elbowJoint != null ? 0.5f * elbowJoint.mass * elbowVelocity * elbowVelocity : 0f,

            magnetPosition = magnet.position,
            magnetVelocity = magnetVelocity,

            boxPosition = movableBox != null ? movableBox.transform.position : boxStartPosition,
            boxVelocity = movableBox != null ? movableBox.velocity : Vector3.zero,

            isBoxAttached = isBoxAttached,
            energyConsumed = energy
        };

        currentPhysicsData.snapshots.Add(snapshot);
    }

    private void CalculateRewards(float energyThisStep)
    {
        // Re-calculate distanceToTarget every frame for smooth physics-based reward
        // IMPORTANT: Use transform.position because Rigidbody.position can freeze when isKinematic and parented to ArticulationBody
        Vector3 boxPos = movableBox != null ? movableBox.transform.position : boxStartPosition;
        
        // Calculate strict horizontal (XZ) distance to effectively guide it towards the drop zone center
        Vector3 boxPosXZ = new Vector3(boxPos.x, 0, boxPos.z);
        Vector3 targetPosXZ = new Vector3(targetPosition.x, 0, targetPosition.z);
        distanceToTarget = Vector3.Distance(boxPosXZ, targetPosXZ);

        // distance improvement reward
        float distanceImprovement = previousDistanceToTarget - distanceToTarget;
        AddReward(distanceImprovement * 2f * rewardMultiplier);
        previousDistanceToTarget = distanceToTarget;

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
        AddReward(-0.001f * rewardMultiplier);  // time penalty: Increased to heavily discourage stalling

        // penalize excessive/jerky movement while holding
        if (isBoxAttached)
        {
            float excessiveMovement = magnetVelocity.magnitude;
            if (excessiveMovement > 2f)
            {
                AddReward(-0.001f * excessiveMovement);
            }
        }

        // postural realism: encourage the elbow to stay above the magnet (like carrying a heavy suspended load)
        if (elbowJoint != null && magnet != null)
        {
            float postureDiff = elbowJoint.transform.position.y - magnet.position.y;
            if (postureDiff < 0)
            {
                // Penalty: the elbow has drooped below the magnet, pushing up awkwardly
                // postureDiff is negative here, so we multiply by a factor to make it a penalty
                AddReward(postureDiff * 0.037f * rewardMultiplier);
            }
            // EXPLOIT FIX: Removed the positive continuous reward for good posture. 
            // In RL, you should only punish bad form, not infinitely reward "not doing the bad form".
            // Otherwise the agent stalls to harvest points!
        }
    }

    private void FixedUpdate()
    {
        if (episodeEnding || movableBox == null) return;

        // Auto-attach by proximity (Fallback to catch skipped physics frames)
        if (!isBoxAttached)
        {
            TryAttachByProximity();
        }
        else
        {
            // Continuously read the exact visual hierarchy position, not the cached Rigidbody.position
            // This prevents the distance check from freezing when the box is parented to an ArticulationBody hierarchy!
            Vector3 boxPos = movableBox.transform.position;
            Vector3 boxPosXZ = new Vector3(boxPos.x, 0, boxPos.z);
            Vector3 targetPosXZ = new Vector3(targetPosition.x, 0, targetPosition.z);
            
            float realHorizontalDistance = Vector3.Distance(boxPosXZ, targetPosXZ);
            distanceToTarget = realHorizontalDistance;

            // Auto-detach when box (carried) reaches zone B — let it land into the trigger
            // extremely tight 0.1m threshold to force dead-center placement before dropping
            if (realHorizontalDistance < 0.1f)
            {
                Debug.Log($"<color=yellow>[AUTO-DETACH] Box at XZ distance {realHorizontalDistance:F3}m, releasing...</color>");
                DetachBox();
                AddReward(3f * rewardMultiplier);  // Boosted intermediate reward for successful delivery
            }
        }
    }

    private void CheckEpisodeEnd()
    {
        // get local floor height for this training area
        float floorY = floor != null ? floor.position.y : transform.position.y;
        
        // magnet hit the floor — should never happen; big penalty + reset
        if (magnet != null && magnet.position.y < floorY + 0.3f)
        {
            AddReward(-10f * rewardMultiplier);
            if (dataCollector != null) CollectEpisodeData(false);
            EndEpisode();
            return;
        }

        // Prevent box from clipping through the floor AND the robot arm itself.
        // Using a spatial overlap query with an "inner core" bypasses collision matrix quirks
        // and guarantees we detect severe clipping without false positives from normal surface resting.
        if (movableBox != null)
        {
            BoxCollider boxCol = movableBox.GetComponent<Collider>() as BoxCollider;
            bool isClipping = false;

            if (boxCol != null)
            {
                Vector3 center = movableBox.transform.TransformPoint(boxCol.center);
                // Scale by 80% creating a 10% safety buffer on all sides of the box. 
                // Any surface resting won't trigger this, but deep clipping will.
                Vector3 halfExtents = Vector3.Scale(boxCol.size, movableBox.transform.lossyScale) * 0.5f * 0.8f;
                
                // Check all solid layers, ignoring triggers (like the magnet trigger or target zones)
                Collider[] overlaps = Physics.OverlapBox(center, halfExtents, movableBox.transform.rotation, ~0, QueryTriggerInteraction.Ignore);
                
                foreach (Collider hit in overlaps)
                {
                    // Ignore overlaps with the box itself
                    if (hit.transform.IsChildOf(movableBox.transform)) continue;
                    
                    isClipping = true;
                    Debug.Log($"<color=red>[CLIPPING FATAL] Box clipped deeply into {hit.name}!</color>");
                    break;
                }
            }

            // Fallback: Also instantly fail if the box somehow gets forced entirely under the floor
            if (isClipping || movableBox.transform.position.y < floorY - 0.5f)
            {
                AddReward(-10f);
                if (dataCollector != null)
                {
                    CollectEpisodeData(false);
                }
                EndEpisode();
                return;
            }
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
        if (joint.jointPosition.dofCount == 0) return; // skip 0-DOF joints (i.e. magnet)

        var drive = joint.xDrive;
        
        // apply action smoothing
        float desiredTarget = drive.target + control * movementSpeed * Time.fixedDeltaTime;
        drive.target = Mathf.Lerp(drive.target, desiredTarget, actionSmoothing);

        bool isFreeMotion = joint.twistLock == ArticulationDofLock.FreeMotion || 
                            joint.swingYLock == ArticulationDofLock.FreeMotion || 
                            joint.swingZLock == ArticulationDofLock.FreeMotion;

        if (!isFreeMotion)
        {
            // joint limit penalty — use ACTUAL joint angle, not drive target
            float actualAngle = GetNormalizedJointAngle(joint);
            if (actualAngle <= drive.lowerLimit + 1f || actualAngle >= drive.upperLimit - 1f)
            {
                AddReward(jointLimitPenalty);
            }

            drive.target = Mathf.Clamp(drive.target, drive.lowerLimit, drive.upperLimit);
        }
        
        joint.xDrive = drive;
    }

    private void ConfigureJointDrives()
    {
        Debug.Log("<color=magenta>=== RobotArm Joint Configuration ===</color>");

        // Diagnostic: Check if joints are assigned
        Debug.Log($"rootBody (Base): {(rootBody != null ? "OK" : "NULL")}");
        Debug.Log($"baseRotation (Body): {(baseRotation != null ? "OK" : "NULL")}");
        Debug.Log($"shoulderJoint (UpperArm): {(shoulderJoint != null ? "OK" : "NULL")}");
        Debug.Log($"elbowJoint (ForeArm): {(elbowJoint != null ? "OK" : "NULL")}");
        Debug.Log($"magnet (Hand): {(magnet != null ? "OK" : "NULL")}");
        
        if (rootBody != null)
        {
            Debug.Log($"<color=white>Root JointType: {rootBody.jointType} (should be FixedJoint or none)</color>");
        }
        
        // Calculate arm scale factor based on distance from base to magnet
        if (magnet != null && baseRotation != null)
        {
            float armLength = Vector3.Distance(baseRotation.transform.position, magnet.position);
            // Reference arm length is 1 unit; scale forces by length^2 (inertia scales with distance squared)
            armScaleFactor = Mathf.Max(1f, armLength * armLength);
            Debug.Log($"<color=cyan>RobotArm Auto-Scale: Arm length = {armLength:F2}, Scale factor = {armScaleFactor:F1}</color>");
        }
        else
        {
            armScaleFactor = 64f; // Default for ~8 unit arm
            Debug.LogWarning("Could not measure arm length, using default scale factor of 64");
        }

        // Apply scaled drive settings to each joint
        ConfigureSingleJointDrive(baseRotation, "Base", true); // True to allow infinite continuous spinning
        ConfigureSingleJointDrive(shoulderJoint, "Shoulder");
        ConfigureSingleJointDrive(elbowJoint, "Elbow");
    }

    private void ConfigureSingleJointDrive(ArticulationBody joint, string jointName, bool allowFreeRotation = false)
    {
        if (joint == null)
        {
            Debug.LogError($"<color=red>{jointName} joint is NULL - cannot configure!</color>");
            return;
        }

        // Log joint type
        Debug.Log($"<color=white>{jointName} JointType: {joint.jointType}</color>");
        
        if (joint.jointType == ArticulationJointType.FixedJoint)
        {
            Debug.LogWarning($"<color=orange>{jointName} is a FIXED joint — auto-converting to RevoluteJoint.</color>");
            joint.jointType = ArticulationJointType.RevoluteJoint;
            joint.twistLock = ArticulationDofLock.LimitedMotion;
        }

        var drive = joint.xDrive;

        // Ensure drive type is set to Target (position-based control)
        drive.driveType = ArticulationDriveType.Target;

        if (allowFreeRotation)
        {
            joint.twistLock = ArticulationDofLock.FreeMotion;
            joint.swingYLock = ArticulationDofLock.FreeMotion;
            joint.swingZLock = ArticulationDofLock.FreeMotion;
            Debug.LogWarning($"<color=cyan>{jointName} set to FreeMotion for infinite optimal rotation.</color>");
        }
        else
        {
            // Fix joint limits if they're zero (common mistake) or restricted to 0-360
            if (drive.lowerLimit == 0 && drive.upperLimit == 0)
            {
                Debug.LogWarning($"<color=orange>{jointName} has [0,0] limits - setting default [-180, 180]</color>");
                drive.lowerLimit = -180f;
                drive.upperLimit = 180f;
            }
            else if (Mathf.Approximately(drive.lowerLimit, 0f) && Mathf.Approximately(drive.upperLimit, 360f))
            {
                Debug.LogWarning($"<color=cyan>{jointName} has [0, 360] limits - re-centering to [-180, 180] for bidirectional optimal rotation</color>");
                drive.lowerLimit = -180f;
                drive.upperLimit = 180f;
            }
        }

        drive.stiffness = baseStiffness * armScaleFactor;
        drive.damping = baseDamping * armScaleFactor;
        drive.forceLimit = baseForceLimit * armScaleFactor;
        joint.xDrive = drive;

        Debug.Log($"<color=yellow>{jointName} Drive: Stiffness={drive.stiffness:F0}, Damping={drive.damping:F0}, ForceLimit={drive.forceLimit:F0}, Limits=[{drive.lowerLimit}, {drive.upperLimit}]</color>");
    }

    private float GetJointAngle(ArticulationBody joint)
    {
        if (joint == null) return 0f;
        if (joint.jointPosition.dofCount == 0) return 0f;
        // use actual joint position, NOT drive target — the target is what the motor
        // is trying to reach, but actual angle can differ due to inertia/load
        return joint.jointPosition[0] * Mathf.Rad2Deg;
    }

    private float GetNormalizedJointAngle(ArticulationBody joint)
    {
        // For joints with FreeMotion, the numerical angle can spin into infinity.
        // This keeps it clamped cleanly between [-180, 180] functionally modding 360 smoothly.
        return Mathf.DeltaAngle(0, GetJointAngle(joint));
    }

    /// Snaps a joint's drive target to its current physical angle.
    /// Call on pickup to prevent spring rebound when box mass leaves the physics sim.
    private void FreezeJointDrivesToCurrent(ArticulationBody joint)
    {
        if (joint == null || joint.jointPosition.dofCount == 0) return;
        var drive = joint.xDrive;
        drive.target = GetJointAngle(joint);
        joint.xDrive = drive;
    }

    private float GetJointVelocity(ArticulationBody joint)
    {
        if (joint == null) return 0f;
        if (joint.jointVelocity.dofCount == 0) return 0f;
        return joint.jointVelocity[0];
    }

    /// Returns an estimate of the applied torque since Unity's jointForce often returns 0 
    /// for position-driven xDrive ArticulationBodies. We estimate the torque based on 
    /// the commanded target velocity and the drive's stiffness/damping parameters.
    private float GetJointTorque(ArticulationBody joint)
    {
        if (joint == null) return 0f;
        if (joint.jointPosition.dofCount == 0 || joint.jointVelocity.dofCount == 0) return 0f;

        // Simplified PD controller torque estimate: 
        // Force = Stiffness * (TargetPos - CurrentPos) - Damping * CurrentVelocity
        float currentPos = joint.jointPosition[0] * Mathf.Rad2Deg; // target is usually in degrees
        float currentVel = joint.jointVelocity[0] * Mathf.Rad2Deg; 
        
        var drive = joint.xDrive;
        float targetPos = drive.target;
        
        float estimatedTorque = drive.stiffness * (targetPos - currentPos) - drive.damping * currentVel;
        
        // Clamp to the physical capabilities of the motor
        return Mathf.Clamp(estimatedTorque, -drive.forceLimit, drive.forceLimit);
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

        // generate start angle randomly (centered around 0 for optimal base rotation)
        float startAngleDeg = Random.Range(-180f, 180f);

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

    private void ResetJoint(ArticulationBody joint, float angleDeg)
    {
        if (joint == null) return;
        var drive = joint.xDrive;
        drive.target = angleDeg;
        joint.xDrive = drive;
        if (joint.jointPosition.dofCount > 0)
        {
            joint.jointPosition = new ArticulationReducedSpace(angleDeg * Mathf.Deg2Rad);
            joint.jointVelocity = new ArticulationReducedSpace(0f);
        }
    }

    private void ResetRobotArm() // reset joints to default positions at episode start
    {
        // TeleportRoot clears the articulation solver's cached velocities/forces,
        // preventing residual momentum from carrying across episodes
        if (rootBody != null)
        {
            rootBody.TeleportRoot(rootBody.transform.position, rootBody.transform.rotation);
        }

        ResetJoint(baseRotation, 0f);
        ResetJoint(shoulderJoint, 45f);
        ResetJoint(elbowJoint, -30f);
    }

    public void OnMagnetTriggerEnter(Collider other) // magnet inbound trigger event handler
    {
        if (other.attachedRigidbody == movableBox && !isBoxAttached)
        {
            AttachBox();
        }
    }

    private void TryAttachByProximity()
    {
        if (isBoxAttached || movableBox == null || magnet == null) return;

        // Slightly relaxed threshold helps under high agent counts and fast arm motion.
        float threshold = magneticRange * 1.2f;
        Collider boxCol = movableBox.GetComponent<Collider>();
        Vector3 closestPoint = boxCol != null ? boxCol.ClosestPoint(magnet.position) : movableBox.transform.position;
        float distance = Vector3.Distance(magnet.position, closestPoint);

        if (distance <= threshold)
        {
            AttachBox();
        }
    }

    private void AttachBox()
    {
        if (isBoxAttached || movableBox == null || episodeEnding) return;
        if (Time.time < attachCooldown) return; // Prevent grabbing during cooldown

        // Snap box so its TOP sits at the magnet — box hangs below the arm tip.
        // Use world-space extents so it works with any box scale.
        float boxHalfHeight = 0.5f;  // fallback for a unit cube
        Collider boxCol = movableBox.GetComponent<Collider>();
        if (boxCol != null) boxHalfHeight = boxCol.bounds.extents.y;

        movableBox.transform.position = magnet.position + Vector3.down * (boxHalfHeight * 1.2f);
        movableBox.velocity = Vector3.zero;
        movableBox.angularVelocity = Vector3.zero;

        // Parent to magnet and make kinematic — moves rigidly with the arm,
        // no FixedJoint break-force issue and no AB solver conflict
        movableBox.transform.SetParent(magnet, true);
        movableBox.isKinematic = true;
        boxParented = true;

        // Freeze drive targets to current angles so joint springs don't rebound.
        // When the box turns kinematic its mass leaves the physics sim — without
        // this the now-unladen spring overshoots and flings the arm upward.
        FreezeJointDrivesToCurrent(baseRotation);
        FreezeJointDrivesToCurrent(shoulderJoint);
        FreezeJointDrivesToCurrent(elbowJoint);

        isBoxAttached = true;
        AddReward(1f * rewardMultiplier);
    }

    private void DetachBox()
    {
        attachCooldown = Time.time + 1.0f; // Prevent re-attaching for 1 second

        if (movableBox != null && boxParented)
        {
            movableBox.transform.SetParent(null, true);
            movableBox.isKinematic = false;
            boxParented = false;
        }
        isBoxAttached = false;
    }

    private void SetBoxColor(Color color)
    {
        // lazy-init: renderer may be on this object or a child _Visual mesh
        if (boxRenderer == null && movableBox != null)
        {
            boxRenderer = movableBox.GetComponent<Renderer>()
                       ?? movableBox.GetComponentInChildren<Renderer>();
            if (boxRenderer != null) boxDefaultColor = boxRenderer.material.color;
        }
        if (boxRenderer != null) boxRenderer.material.color = color;
    }

    /// Holds the green success colour for a brief moment, raises the arm, then ends the episode.
    private System.Collections.IEnumerator SuccessFlashThenEnd()
    {
        episodeEnding = true;
        
        float endTime = Time.realtimeSinceStartup + 0.6f;
        while (Time.realtimeSinceStartup < endTime)
        {
            // Raise the arm up slightly by driving joints toward their lower/negative limits (usually UP/BACK)
            if (shoulderJoint != null) ApplyJointTorque(shoulderJoint, -1f);
            if (elbowJoint != null) ApplyJointTorque(elbowJoint, -1f);
            
            yield return new WaitForFixedUpdate();
        }

        EndEpisode();
    }

    // OnJointBreak no longer needed — removed FixedJoint approach

    void OnDrawGizmos() // visualize magnet range 
    {
        if (!visualizeMagnetRange || magnet == null) return;

        Gizmos.color = isBoxAttached ? Color.green : Color.yellow;
        Gizmos.DrawWireSphere(magnet.position, magneticRange);

        if (movableBox != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(magnet.position, movableBox.transform.position);
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
        if (isBoxAttached) GUILayout.Label($"Distance to Goal (Landing Area): {Vector3.Distance(movableBox != null ? movableBox.transform.position : boxStartPosition, targetPosition):F2}m");
        else GUILayout.Label($"Distance to Goal (Box): {Vector3.Distance(magnet.position, movableBox != null ? movableBox.transform.position : boxStartPosition):F2}m");

        GUILayout.Label($"Energy Used: {totalEnergyConsumed:F2}");
        if (usePowerBudget) GUILayout.Label($"Power: {currentPower / maxPowerBudget * 100f:F1}%");
        GUILayout.Label($"Action Smoothing: {actionSmoothing:F2}");
        GUILayout.Label($"");
        GUILayout.Label($"Joint Angles:");
        GUILayout.Label($"  Base: {GetNormalizedJointAngle(baseRotation):F1}°");
        GUILayout.Label($"  Shoulder: {GetNormalizedJointAngle(shoulderJoint):F1}°");
        GUILayout.Label($"  Elbow: {GetNormalizedJointAngle(elbowJoint):F1}°");
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
