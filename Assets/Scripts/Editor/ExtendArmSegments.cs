using UnityEngine;
using UnityEditor;

/// <summary>
/// Incrementally increases UpperArm and Forearm segment lengths.
/// Adjusts: local positions, visual scales, elbow/magnet positions, colliders.
/// Run via  Tools ▸ Extend Arm Segments
/// </summary>
public static class ExtendArmSegments
{
    // === TWEAK THESE TO ADJUST LENGTH ===
    // Negative = trim. Set back to 0 after applying to avoid double-application.
    // Last applied: -0.2 trim on both. Current: Forearm=2.7, Magnet=1.8
    static float upperArmExtension = 0.0f;  // added to UpperArm segment (Forearm localPos.y)
    static float forearmExtension  = 0.0f;  // added to Forearm segment (Magnet localPos.y)

    [MenuItem("Tools/Extend Arm Segments")]
    static void Execute()
    {
        const string prefabPath = "Assets/Prefabs/TrainingArea_0.prefab";
        GameObject root = PrefabUtility.LoadPrefabContents(prefabPath);
        if (root == null) { Debug.LogError("Prefab not found!"); return; }

        Transform robotArm = root.transform.Find("RobotArm");
        Transform baseRot  = robotArm.Find("BaseRotation");
        Transform body     = baseRot.Find("Body");
        Transform upperArm = body.Find("UpperArm");
        Transform forearm  = upperArm.Find("Forearm");
        Transform magnet   = forearm.Find("Magnet");

        // Cosmetic joints
        Transform elbowJoint = upperArm.Find("ElbowJoint");

        // Visual children
        Transform upperArmVis = upperArm.Find("UpperArm_Visual");
        Transform forearmVis  = forearm.Find("Forearm_Visual");

        // ── Log before ──
        Debug.Log("<color=cyan>=== BEFORE EXTENSION ===</color>");
        Debug.Log($"  UpperArm segment (Forearm localPos.y): {forearm.localPosition.y:F3}");
        Debug.Log($"  Forearm segment (Magnet localPos.y):   {magnet.localPosition.y:F3}");
        Debug.Log($"  Total reach from shoulder:             {forearm.localPosition.y + magnet.localPosition.y:F3}");
        Debug.Log($"  ElbowJoint localPos.y:                 {(elbowJoint != null ? elbowJoint.localPosition.y.ToString("F3") : "N/A")}");

        // ── 1. Extend UpperArm segment ──
        // Move Forearm further from UpperArm pivot
        Vector3 fPos = forearm.localPosition;
        fPos.y += upperArmExtension;
        forearm.localPosition = fPos;

        // Move ElbowJoint to stay at the midpoint/proportional position
        if (elbowJoint != null)
        {
            Vector3 ePos = elbowJoint.localPosition;
            // ElbowJoint was at 0.6 when segment was 1.2 (50% along)
            // Keep it proportional
            float oldSegLen = fPos.y - upperArmExtension;
            float ratio = ePos.y / oldSegLen;
            ePos.y = ratio * fPos.y;
            elbowJoint.localPosition = ePos;
        }

        // Scale UpperArm visual to match new length
        if (upperArmVis != null)
        {
            Vector3 vs = upperArmVis.localScale;
            float oldSegLen = fPos.y - upperArmExtension;
            vs.y *= fPos.y / oldSegLen;  // scale proportionally
            upperArmVis.localScale = vs;
        }

        // Resize UpperArm BoxCollider
        BoxCollider upperArmCol = upperArm.GetComponent<BoxCollider>();
        if (upperArmCol != null)
        {
            Vector3 sz = upperArmCol.size;
            float oldSegLen = fPos.y - upperArmExtension;
            sz.y *= fPos.y / oldSegLen;
            upperArmCol.size = sz;
        }

        // ── 2. Extend Forearm segment ──
        // Move Magnet further from Forearm pivot
        Vector3 mPos = magnet.localPosition;
        mPos.y += forearmExtension;
        magnet.localPosition = mPos;

        // Scale Forearm visual to match new length
        if (forearmVis != null)
        {
            Vector3 vs = forearmVis.localScale;
            float oldSegLen = mPos.y - forearmExtension;
            vs.y *= mPos.y / oldSegLen;
            forearmVis.localScale = vs;
        }

        // Resize Forearm BoxCollider
        BoxCollider forearmCol = forearm.GetComponent<BoxCollider>();
        if (forearmCol != null)
        {
            Vector3 sz = forearmCol.size;
            float oldSegLen = mPos.y - forearmExtension;
            sz.y *= mPos.y / oldSegLen;
            forearmCol.size = sz;
        }

        // ── Log after ──
        Debug.Log("<color=yellow>=== AFTER EXTENSION ===</color>");
        Debug.Log($"  UpperArm segment (Forearm localPos.y): {forearm.localPosition.y:F3}");
        Debug.Log($"  Forearm segment (Magnet localPos.y):   {magnet.localPosition.y:F3}");
        Debug.Log($"  Total reach from shoulder:             {forearm.localPosition.y + magnet.localPosition.y:F3}");
        Debug.Log($"  ElbowJoint localPos.y:                 {(elbowJoint != null ? elbowJoint.localPosition.y.ToString("F3") : "N/A")}");
        Debug.Log($"  Magnet world Y (approx):               {magnet.position.y:F3}");

        // ── Save ──
        PrefabUtility.SaveAsPrefabAsset(root, prefabPath);
        PrefabUtility.UnloadPrefabContents(root);

        Debug.Log("<color=green>[ExtendArm] Done! Prefab saved.</color>");
        Debug.Log("<color=green>If too short/long, change values in ExtendArmSegments.cs and run again.</color>");
    }
}
