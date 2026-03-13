using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

/// <summary>
/// One-click fix: sets every ArticulationBody object's scale to (1,1,1),
/// moves meshes into "_Visual" children so they keep their appearance,
/// resizes colliders to match, and preserves all world positions.
/// 
/// Run via  Tools ▸ Fix Robot Arm Scales
/// Then run Tools ▸ Dump Robot Arm Transforms  to verify.
/// </summary>
public static class FixRobotArmScales
{
    struct Snap
    {
        public Vector3 worldPos;
        public Quaternion worldRot;
        public Vector3 lossyScale;
    }

    [MenuItem("Tools/Fix Robot Arm Scales")]
    static void Execute()
    {
        const string prefabPath = "Assets/Prefabs/TrainingArea_0.prefab";
        GameObject root = PrefabUtility.LoadPrefabContents(prefabPath);
        if (root == null) { Debug.LogError("Prefab not found at " + prefabPath); return; }

        Transform robotArm = root.transform.Find("RobotArm");
        if (robotArm == null)
        {
            Debug.LogError("RobotArm child not found in prefab!");
            PrefabUtility.UnloadPrefabContents(root);
            return;
        }

        // ── 1. Snapshot every transform (world pos, rotation, lossy scale) ──
        var snaps = new Dictionary<Transform, Snap>();
        RecordAll(robotArm, snaps);

        Debug.Log($"<color=cyan>[FixScales] Recorded {snaps.Count} transforms.</color>");

        // ── 2. For AB objects that also carry a mesh, split mesh → child ──
        var visualChildren = new List<(Transform abObj, Vector3 meshLossyScale)>();
        SplitMeshesFromAB(robotArm, snaps, visualChildren);

        Debug.Log($"<color=cyan>[FixScales] Created {visualChildren.Count} visual children.</color>");

        // ── 3. Set RobotArm and every AB to scale (1,1,1) ──
        robotArm.localScale = Vector3.one;
        SetABScalesToOne(robotArm);

        // ── 4. Restore world positions top-down (parents before children) ──
        RestoreWorldPositions(robotArm, snaps);

        // ── 5. Set each visual child's local scale = old lossy scale ──
        //       (all ancestors are now (1,1,1) so local scale = world scale)
        foreach (var (abObj, meshScale) in visualChildren)
        {
            Transform vis = abObj.Find(abObj.name + "_Visual");
            if (vis != null)
            {
                vis.localScale = meshScale;
                Debug.Log($"  {vis.name} scale → ({meshScale.x:F4}, {meshScale.y:F4}, {meshScale.z:F4})");
            }
        }

        // ── 6. Fix non-AB mesh objects (ShoulderJoint, ElbowJoint, Magnet) ──
        //       Their parent chain changed, so set localScale = old lossyScale
        FixNonABMeshScales(robotArm, snaps);

        // ── 7. Resize colliders on AB objects to maintain physics shape ──
        ResizeABColliders(robotArm, snaps);

        // ── 8. Log final state ──
        Debug.Log("<color=yellow>=== AFTER FIX ===</color>");
        LogHierarchy(robotArm, 0);

        // ── 9. Save prefab ──
        PrefabUtility.SaveAsPrefabAsset(root, prefabPath);
        PrefabUtility.UnloadPrefabContents(root);

        Debug.Log("<color=green>[FixScales] Done! Prefab saved. All AB scales are now (1,1,1).</color>");
        Debug.Log("<color=green>Run 'Tools > Dump Robot Arm Transforms' to double-check.</color>");
    }

    // ─────────── helpers ───────────

    static void RecordAll(Transform t, Dictionary<Transform, Snap> dict)
    {
        dict[t] = new Snap
        {
            worldPos = t.position,
            worldRot = t.rotation,
            lossyScale = t.lossyScale
        };
        for (int i = 0; i < t.childCount; i++)
            RecordAll(t.GetChild(i), dict);
    }

    static void SplitMeshesFromAB(Transform t, Dictionary<Transform, Snap> snaps,
                                   List<(Transform, Vector3)> visualList)
    {
        // Process children first (bottom-up) so child indices don't shift
        for (int i = 0; i < t.childCount; i++)
            SplitMeshesFromAB(t.GetChild(i), snaps, visualList);

        var ab = t.GetComponent<ArticulationBody>();
        var mf = t.GetComponent<MeshFilter>();
        var mr = t.GetComponent<MeshRenderer>();

        if (ab != null && mf != null && mr != null)
        {
            // Create visual child at origin
            GameObject vis = new GameObject(t.name + "_Visual");
            vis.transform.SetParent(t, false);           // local pos/rot = identity
            vis.transform.localPosition = Vector3.zero;
            vis.transform.localRotation = Quaternion.identity;

            // Copy mesh components
            MeshFilter newMF = vis.AddComponent<MeshFilter>();
            newMF.sharedMesh = mf.sharedMesh;

            MeshRenderer newMR = vis.AddComponent<MeshRenderer>();
            newMR.sharedMaterials = mr.sharedMaterials;
            newMR.shadowCastingMode = mr.shadowCastingMode;
            newMR.receiveShadows = mr.receiveShadows;

            // Remove originals (MeshRenderer first, it depends on MeshFilter)
            Object.DestroyImmediate(mr);
            Object.DestroyImmediate(mf);

            // Record for step 5
            visualList.Add((t, snaps[t].lossyScale));

            Debug.Log($"  Split mesh from {t.name} (lossy was {snaps[t].lossyScale})");
        }
    }

    static void SetABScalesToOne(Transform t)
    {
        if (t.GetComponent<ArticulationBody>() != null)
            t.localScale = Vector3.one;

        for (int i = 0; i < t.childCount; i++)
            SetABScalesToOne(t.GetChild(i));
    }

    static void RestoreWorldPositions(Transform t, Dictionary<Transform, Snap> snaps)
    {
        // Only restore objects we recorded (skip newly created _Visual children)
        if (snaps.ContainsKey(t))
        {
            t.position = snaps[t].worldPos;
            t.rotation = snaps[t].worldRot;
        }

        for (int i = 0; i < t.childCount; i++)
            RestoreWorldPositions(t.GetChild(i), snaps);
    }

    static void FixNonABMeshScales(Transform t, Dictionary<Transform, Snap> snaps)
    {
        // Non-AB objects that have meshes (ShoulderJoint, ElbowJoint, Magnet):
        // Their parent compound scale changed, so set localScale = old lossyScale
        // to preserve their visual size (ancestors are all (1,1,1) now)
        bool hasAB = t.GetComponent<ArticulationBody>() != null;
        bool hasMesh = t.GetComponent<MeshFilter>() != null;

        if (!hasAB && hasMesh && snaps.ContainsKey(t))
        {
            Vector3 oldLossy = snaps[t].lossyScale;
            t.localScale = oldLossy;
            Debug.Log($"  {t.name} localScale → ({oldLossy.x:F4}, {oldLossy.y:F4}, {oldLossy.z:F4})");
        }

        for (int i = 0; i < t.childCount; i++)
            FixNonABMeshScales(t.GetChild(i), snaps);
    }

    static void ResizeABColliders(Transform t, Dictionary<Transform, Snap> snaps)
    {
        if (t.GetComponent<ArticulationBody>() != null && snaps.ContainsKey(t))
        {
            Vector3 ls = snaps[t].lossyScale; // old lossy scale

            // BoxCollider: world size = size * lossyScale → new size = old size * old lossy
            BoxCollider box = t.GetComponent<BoxCollider>();
            if (box != null)
            {
                box.size = Vector3.Scale(box.size, ls);
                box.center = Vector3.Scale(box.center, ls);
                Debug.Log($"  {t.name} BoxCollider resized → size {box.size}");
            }

            // CapsuleCollider: direction-dependent scaling
            CapsuleCollider cap = t.GetComponent<CapsuleCollider>();
            if (cap != null)
            {
                int dir = cap.direction;
                float heightScale, radiusScale;

                if (dir == 0)      { heightScale = ls.x; radiusScale = Mathf.Max(ls.y, ls.z); }
                else if (dir == 1) { heightScale = ls.y; radiusScale = Mathf.Max(ls.x, ls.z); }
                else               { heightScale = ls.z; radiusScale = Mathf.Max(ls.x, ls.y); }

                cap.height *= heightScale;
                cap.radius *= radiusScale;
                cap.center = Vector3.Scale(cap.center, ls);
                Debug.Log($"  {t.name} CapsuleCollider resized → r={cap.radius:F3}, h={cap.height:F3}");
            }

            // SphereCollider: uniform max scale
            SphereCollider sph = t.GetComponent<SphereCollider>();
            if (sph != null)
            {
                float maxScale = Mathf.Max(ls.x, Mathf.Max(ls.y, ls.z));
                sph.radius *= maxScale;
                sph.center = Vector3.Scale(sph.center, ls);
                Debug.Log($"  {t.name} SphereCollider resized → r={sph.radius:F3}");
            }
        }

        for (int i = 0; i < t.childCount; i++)
            ResizeABColliders(t.GetChild(i), snaps);
    }

    static void LogHierarchy(Transform t, int depth)
    {
        string pad = new string(' ', depth * 2);
        string tags = "";
        if (t.GetComponent<ArticulationBody>()) tags += "[AB]";
        if (t.GetComponent<MeshFilter>()) tags += "[Mesh]";
        if (t.GetComponent<MeshRenderer>()) tags += "[MR]";

        Debug.Log($"{pad}{t.name} {tags}" +
            $"  pos=({t.localPosition.x:F3},{t.localPosition.y:F3},{t.localPosition.z:F3})" +
            $"  scale=({t.localScale.x:F4},{t.localScale.y:F4},{t.localScale.z:F4})" +
            $"  world=({t.position.x:F3},{t.position.y:F3},{t.position.z:F3})");

        for (int i = 0; i < t.childCount; i++)
            LogHierarchy(t.GetChild(i), depth + 1);
    }
}
