using UnityEngine;
using UnityEditor;

/// <summary>
/// Snaps ArticulationBody anchor positions to the visual joint sphere world positions.
/// - UpperArm AB: anchors → ShoulderJoint sphere position
/// - Forearm   AB: anchors → ElbowJoint  sphere position
///
/// Run via  Tools ▸ Fix Anchor Positions
/// </summary>
public static class FixAnchorPositions
{
    [MenuItem("Tools/Fix Anchor Positions")]
    static void Execute()
    {
        const string prefabPath = "Assets/Prefabs/TrainingArea_0.prefab";
        GameObject root = PrefabUtility.LoadPrefabContents(prefabPath);
        if (root == null) { Debug.LogError("[FixAnchors] Prefab not found!"); return; }

        // ── Resolve hierarchy ──
        Transform robotArm    = root.transform.Find("RobotArm");
        Transform baseRot     = robotArm.Find("BaseRotation");
        Transform body        = baseRot.Find("Body");
        Transform upperArm    = body.Find("UpperArm");
        Transform forearm     = upperArm.Find("Forearm");

        // Cosmetic joint spheres
        Transform shoulderJoint = body.Find("ShoulderJoint");
        Transform elbowJoint    = upperArm.Find("ElbowJoint");

        if (shoulderJoint == null) { Debug.LogError("[FixAnchors] ShoulderJoint not found under Body!"); PrefabUtility.UnloadPrefabContents(root); return; }
        if (elbowJoint    == null) { Debug.LogError("[FixAnchors] ElbowJoint not found under UpperArm!"); PrefabUtility.UnloadPrefabContents(root); return; }

        ArticulationBody upperArmAB = upperArm.GetComponent<ArticulationBody>();
        ArticulationBody forearmAB  = forearm.GetComponent<ArticulationBody>();

        if (upperArmAB == null) { Debug.LogError("[FixAnchors] UpperArm has no ArticulationBody!"); PrefabUtility.UnloadPrefabContents(root); return; }
        if (forearmAB  == null) { Debug.LogError("[FixAnchors] Forearm has no ArticulationBody!");  PrefabUtility.UnloadPrefabContents(root); return; }

        // ── UpperArm anchors → ShoulderJoint position ──
        Vector3 shoulderWorldPos = shoulderJoint.position;

        Vector3 upperArmParentAnchor = body.InverseTransformPoint(shoulderWorldPos);      // in Body's local space
        Vector3 upperArmChildAnchor  = upperArm.InverseTransformPoint(shoulderWorldPos);  // in UpperArm's local space

        Debug.Log($"[FixAnchors] ShoulderJoint world pos: {shoulderWorldPos}");
        Debug.Log($"[FixAnchors] UpperArm parentAnchor (was {upperArmAB.parentAnchorPosition}) → {upperArmParentAnchor}");
        Debug.Log($"[FixAnchors] UpperArm anchorPos    (was {upperArmAB.anchorPosition})       → {upperArmChildAnchor}");

        upperArmAB.parentAnchorPosition = upperArmParentAnchor;
        upperArmAB.anchorPosition       = upperArmChildAnchor;

        // ── Forearm anchors → ElbowJoint position ──
        Vector3 elbowWorldPos = elbowJoint.position;

        Vector3 forearmParentAnchor = upperArm.InverseTransformPoint(elbowWorldPos);  // in UpperArm's local space
        Vector3 forearmChildAnchor  = forearm.InverseTransformPoint(elbowWorldPos);   // in Forearm's local space

        Debug.Log($"[FixAnchors] ElbowJoint world pos: {elbowWorldPos}");
        Debug.Log($"[FixAnchors] Forearm parentAnchor (was {forearmAB.parentAnchorPosition}) → {forearmParentAnchor}");
        Debug.Log($"[FixAnchors] Forearm anchorPos    (was {forearmAB.anchorPosition})       → {forearmChildAnchor}");

        forearmAB.parentAnchorPosition = forearmParentAnchor;
        forearmAB.anchorPosition       = forearmChildAnchor;

        // ── Save ──
        PrefabUtility.SaveAsPrefabAsset(root, prefabPath);
        PrefabUtility.UnloadPrefabContents(root);

        Debug.Log("<color=green>[FixAnchors] Done! Anchor positions saved to prefab.</color>");
        Debug.Log("<color=green>Open the prefab in Inspector to verify the Articulation Body anchor gizmos sit on the joint spheres.</color>");
    }
}
