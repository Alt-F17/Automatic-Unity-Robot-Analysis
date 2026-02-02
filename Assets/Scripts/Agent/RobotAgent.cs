using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace AutomaticUnityRobotAnalysis
{
    public class RobotAgent : Agent
    {
        [Header("Robot Joint Components")]
        [SerializeField] private ArticulationBody baseRotation;
        [SerializeField] private ArticulationBody shoulderJoint;
        [SerializeField] private ArticulationBody elbowJoint;
        [SerializeField] private Transform footMagnet;

        [Header("Environment Objects")]
        [SerializeField] private readonly Rigidbody movableBox;
        [SerializeField] private readonly Transform targetZoneA;
        [SerializeField] private readonly Transform targetZoneB;
        [SerializeField] private readonly Transform floor;

        [Header("Magnet Settings")]
        [SerializeField] private readonly float magneticRange = 0.5f;
        [SerializeField] private readonly float magneticStrength = 100f;
        [SerializeField] private readonly bool visualizeMagnetRange = true;

        [Header("Training Parameters")]
        [SerializeField] private readonly float maxMotorForce = 100f;
        [SerializeField] private readonly float movementSpeed = 50f;
        [SerializeField] private readonly float rewardMultiplier = 1f;

        [Header("Power Budget System")]
        [SerializeField] private bool usePowerBudget = false;
        [SerializeField] private readonly float maxPowerBudget = 100f;
        [SerializeField] private float currentPower = 100f;

        [Header("Random Position System")]
        [SerializeField] private bool useRandomPositions = false;
        [SerializeField] private readonly float minDistance = 2f;
        [SerializeField] private readonly float workspaceRadius = 3.5f;

        [Header("Suggestion 1: Curriculum Settings")]
        [Tooltip("After this many episodes, enable Power Budget and Random Positions")]
        [SerializeField] private readonly int curriculumEpisodeThreshold = 500;
        private bool curriculumActive = false;

        [Header("Suggestion 2: Safety Zones")]
        [Tooltip("Penalty applied per frame when a joint is pushed against its physical limit")]
        [SerializeField] private readonly float jointLimitPenalty = -0.001f;

        [Header("Suggestion 4: Action Smoothing")]
        [Range(0.01f, 1f)]
        [Tooltip("Lower = smoother/heavier movement, Higher = snappier/instant movement")]
        [SerializeField] private readonly float actionSmoothing = 0.15f;

        // Internal State
        private bool isBoxAttached = false;
        private FixedJoint magnetJoint;
        private Vector3 boxStartPosition;
        private Vector3 targetPosition;
        private float episodeStartTime;
        private float totalEnergyConsumed;
        private int successfulMoves;
        private int totalAttempts;
        private float distanceToTarget;
        private float previousDistanceToTarget;
        private Vector3 previousFootPosition;
        private Vector3 footVelocity;

        public override void Initialize()
        {
            boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
            targetPosition = targetZoneB.position + Vector3.up * 0.5f;
            SetupMagnetCollider();
        }

        private void SetupMagnetCollider()
        {
            SphereCollider magnetCollider = footMagnet.gameObject.GetComponent<SphereCollider>() ?? footMagnet.gameObject.AddComponent<SphereCollider>();
            magnetCollider.radius = magneticRange;
            magnetCollider.isTrigger = true;
            
            MagnetTrigger trigger = footMagnet.gameObject.GetComponent<MagnetTrigger>();
            trigger ??= footMagnet.gameObject.AddComponent<MagnetTrigger>();
            trigger.Initialize(this);
        }

        public override void OnEpisodeBegin()
        {
            if (CompletedEpisodes >= curriculumEpisodeThreshold && !curriculumActive)
            {
                curriculumActive = true;
                usePowerBudget = true;
                useRandomPositions = true;
                Debug.Log("<color=green>Curriculum Phase 2: Power Budget and Randomization Enabled!</color>");
            }

            DetachBox();
            ResetRobotArm();

            if (usePowerBudget) currentPower = maxPowerBudget;

            if (useRandomPositions)
            {
                GenerateRandomPositions();
            }
            else
            {
                boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
                targetPosition = targetZoneB.position + Vector3.up * 0.5f;
                if (targetZoneA) targetZoneA.position = new Vector3(boxStartPosition.x, 0.05f, boxStartPosition.z);
                if (targetZoneB) targetZoneB.position = new Vector3(targetPosition.x, 0.05f, targetPosition.z);
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
            previousFootPosition = footMagnet.position;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            sensor.AddObservation(transform.InverseTransformPoint(footMagnet.position));
            sensor.AddObservation(transform.InverseTransformPoint(movableBox.position));
            sensor.AddObservation(transform.InverseTransformPoint(targetPosition));

            distanceToTarget = Vector3.Distance(movableBox.position, targetPosition);
            sensor.AddObservation(distanceToTarget);
            sensor.AddObservation(GetJointAngle(baseRotation) / 180f);
            sensor.AddObservation(GetJointAngle(shoulderJoint) / 180f);
            sensor.AddObservation(GetJointAngle(elbowJoint) / 180f);

            sensor.AddObservation(isBoxAttached ? 1f : 0f);
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

            if (usePowerBudget)
            {
                currentPower -= energyThisStep;
                if (currentPower <= 0f)
                {
                    AddReward(-5f);
                    EndEpisode();
                    return;
                }
            }

            CalculateRewards();
            CheckEpisodeEnd();
        }

        private void ApplyJointTorque(ArticulationBody joint, float control)
        {
            if (joint == null) return;

            var drive = joint.xDrive;

            float desiredTarget = drive.target + control * movementSpeed * Time.fixedDeltaTime;
            drive.target = Mathf.Lerp(drive.target, desiredTarget, actionSmoothing);

            if (drive.target <= drive.lowerLimit + 1f || drive.target >= drive.upperLimit - 1f)
            {
                AddReward(jointLimitPenalty);
            }

            drive.target = Mathf.Clamp(drive.target, drive.lowerLimit, drive.upperLimit);
            joint.xDrive = drive;
        }

        private void CalculateRewards()
        {
            float currentDistance = Vector3.Distance(movableBox.position, targetPosition);
            float improvement = previousDistanceToTarget - currentDistance;
            AddReward(improvement * 2f * rewardMultiplier);
            previousDistanceToTarget = currentDistance;

            if (isBoxAttached)
            {
                AddReward(0.02f);
            }

            if (currentDistance < 0.5f)
            {
                float bonus = 10f;
                if (usePowerBudget) bonus += currentPower / maxPowerBudget * 10f;
                AddReward(bonus);
                successfulMoves++;
                EndEpisode();
            }
        }

        private void CheckEpisodeEnd()
        {
            if (movableBox.position.y < floor.position.y - 0.5f) { AddReward(-5f); EndEpisode(); }
            if (Time.time - episodeStartTime > 60f) EndEpisode();
        }

        private float CalculateEnergyConsumption(float b, float s, float e)
        {
            return (Mathf.Abs(b) + Mathf.Abs(s) + Mathf.Abs(e)) * 0.01f;
        }

        private static float GetJointAngle(ArticulationBody joint) => joint ? joint.xDrive.target : 0f;

        private void GenerateRandomPositions()
        {
            float startAngle = Random.Range(0f, 360f) * Mathf.Deg2Rad;
            float startRadius = Random.Range(1.5f, workspaceRadius);
            boxStartPosition = new Vector3(Mathf.Cos(startAngle) * startRadius, 0.75f, Mathf.Sin(startAngle) * startRadius);

            float endAngle = Random.Range(0f, 360f) * Mathf.Deg2Rad;
            float endRadius = Random.Range(1.5f, workspaceRadius);
            targetPosition = new Vector3(Mathf.Cos(endAngle) * endRadius, 0.75f, Mathf.Sin(endAngle) * endRadius);

            if (targetZoneA) targetZoneA.position = new Vector3(boxStartPosition.x, 0.05f, boxStartPosition.z);
            if (targetZoneB) targetZoneB.position = new Vector3(targetPosition.x, 0.05f, targetPosition.z);
        }

        private void ResetRobotArm()
        {
            ResetJoint(baseRotation, 0f);
            ResetJoint(shoulderJoint, 45f);
            ResetJoint(elbowJoint, -30f);
        }

        private static void ResetJoint(ArticulationBody joint, float angle)
        {
            if (!joint) return;
            var drive = joint.xDrive;
            drive.target = angle;
            joint.xDrive = drive;
            joint.jointVelocity = new ArticulationReducedSpace(0f);
        }

        public void OnMagnetTriggerEnter(Collider other)
        {
            if (other.attachedRigidbody == movableBox && !isBoxAttached) AttachBox();
        }

        private void AttachBox()
        {
            if (magnetJoint) return;
            magnetJoint = footMagnet.gameObject.AddComponent<FixedJoint>();
            magnetJoint.connectedBody = movableBox;
            magnetJoint.breakForce = magneticStrength;
            isBoxAttached = true;
            AddReward(1.0f);
        }

        private void DetachBox() { if (magnetJoint) Destroy(magnetJoint); isBoxAttached = false; }

        void OnGUI()
        {
            if (!Application.isPlaying) return;
            GUI.Box(new Rect(10, 10, 250, 150), "Robot Intelligence Status");
            GUILayout.BeginArea(new Rect(20, 35, 230, 140));
            GUILayout.Label($"Curriculum: {(curriculumActive ? "Phase 2 (Hard)" : "Phase 1 (Easy)")}");
            GUILayout.Label($"Success Rate: {(float)successfulMoves / totalAttempts * 100f:F1}%");
            if (usePowerBudget) GUILayout.Label($"Power: {currentPower / maxPowerBudget * 100f:F1}%");
            GUILayout.Label($"Smoothing: {actionSmoothing:F2}");
            GUILayout.EndArea();
        }
    }

    public class MagnetTrigger : MonoBehaviour
    {
        private RobotAgent agent;
        public void Initialize(RobotAgent a) => agent = a;
        void OnTriggerEnter(Collider other) => agent?.OnMagnetTriggerEnter(other);
    }
}
