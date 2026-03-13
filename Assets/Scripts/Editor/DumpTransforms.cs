using UnityEngine;
using UnityEditor;

public class DumpTransforms : MonoBehaviour
{
    [MenuItem("Tools/Dump Robot Arm Transforms")]
    static void Dump()
    {
        // Load the prefab
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/TrainingArea_0.prefab");
        if (prefab == null)
        {
            Debug.LogError("Could not find TrainingArea_0 prefab!");
            return;
        }

        Debug.Log("=== ROBOT ARM TRANSFORM DUMP ===");
        
        // Recursively dump all transforms
        DumpRecursive(prefab.transform, 0);
        
        Debug.Log("=== END DUMP ===");
    }

    static void DumpRecursive(Transform t, int depth)
    {
        string indent = new string(' ', depth * 2);
        string components = "";
        
        if (t.GetComponent<ArticulationBody>() != null)
            components += " [AB]";
        if (t.GetComponent<MeshFilter>() != null)
            components += " [Mesh]";
        if (t.GetComponent<Rigidbody>() != null)
            components += " [RB]";
        if (t.GetComponent<Collider>() != null)
            components += " [Col]";

        Debug.Log($"{indent}{t.name}{components}" +
                  $"\n{indent}  LocalPos: ({t.localPosition.x:F4}, {t.localPosition.y:F4}, {t.localPosition.z:F4})" +
                  $"\n{indent}  LocalRot: ({t.localEulerAngles.x:F1}, {t.localEulerAngles.y:F1}, {t.localEulerAngles.z:F1})" +
                  $"\n{indent}  Scale:    ({t.localScale.x:F4}, {t.localScale.y:F4}, {t.localScale.z:F4})" +
                  $"\n{indent}  WorldPos: ({t.position.x:F4}, {t.position.y:F4}, {t.position.z:F4})");

        for (int i = 0; i < t.childCount; i++)
        {
            DumpRecursive(t.GetChild(i), depth + 1);
        }
    }
}
