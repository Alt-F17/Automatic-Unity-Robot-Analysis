using UnityEngine;
using UnityEditor;
using System.IO;

public class InteractiveDemoBuilder
{
    [MenuItem("AURA/Build Interactive Demo Scene")]
    public static void BuildScene()
    {
        // 1. Create a new scene
        var newScene = UnityEditor.SceneManagement.EditorSceneManager.NewScene(UnityEditor.SceneManagement.NewSceneSetup.DefaultGameObjects, UnityEditor.SceneManagement.NewSceneMode.Single);
        
        // 2. Load the Robot Prefab
        GameObject robotPrefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/TrainingArea_Robot.prefab");
        if (robotPrefab == null)
        {
            Debug.LogError("Could not find TrainingArea_Robot prefab. Cannot build scene automatically.");
            return;
        }

        // 3. Instantiate and strip training components
        GameObject environment = PrefabUtility.InstantiatePrefab(robotPrefab) as GameObject;
        environment.name = "Interactive_Robot_Environment";
        PrefabUtility.UnpackPrefabInstance(environment, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);

        // Find the actual robot arm base
        Transform armBase = environment.transform.Find("RobotArm_Base");
        if (armBase == null)
        {
            Debug.LogError("Could not find RobotArm_Base inside the prefab.");
            return;
        }

        // 4. Swap the Scripts on the Agent
        RobotAgent oldAgent = armBase.GetComponent<RobotAgent>();
        
        InteractiveRobotAgent newAgent = armBase.gameObject.AddComponent<InteractiveRobotAgent>();
        
        // Copy the exact parameters over
        newAgent.MaxStep = oldAgent.MaxStep;
        
        // Use SerializedObject to copy privately serialized fields
        SerializedObject oldObj = new SerializedObject(oldAgent);
        SerializedObject newObj = new SerializedObject(newAgent);
        
        SerializedProperty prop = oldObj.GetIterator();
        bool enterChildren = true;
        while (prop.NextVisible(enterChildren))
        {
            enterChildren = false;
            // Skip script and base ML-agents properties that we can't easily overwrite directly
            if (prop.name == "m_Script" || prop.name == "m_BehaviorParameters" || prop.name == "m_BrainParameters") continue;
            
            SerializedProperty newProp = newObj.FindProperty(prop.name);
            if (newProp != null && newProp.propertyType == prop.propertyType)
            {
                newProp.serializedObject.CopyFromSerializedPropertyIfDifferent(prop);
            }
        }
        newObj.ApplyModifiedProperties();
        newAgent.isInteractiveMode = true; // explicitly enable
        Object.DestroyImmediate(oldAgent);

        // 5. Setup Interactive Zones
        Transform zoneA = environment.transform.Find("Zone_Start");
        Transform zoneB = environment.transform.Find("Zone_Target");
        
        if (zoneA)
        {
            var tc = zoneA.gameObject.AddComponent<InteractiveZoneController>();
            tc.robotBase = armBase;
            tc.otherZone = zoneB;
            // Add a collider so it can be clicked/dragged
            var coll = zoneA.gameObject.GetComponent<BoxCollider>();
            if (!coll) coll = zoneA.gameObject.AddComponent<BoxCollider>();
            coll.size = new Vector3(1f, 0.1f, 1f);
        }
        if (zoneB)
        {
            var tc = zoneB.gameObject.AddComponent<InteractiveZoneController>();
            tc.robotBase = armBase;
            tc.otherZone = zoneA;
            var coll = zoneB.gameObject.GetComponent<BoxCollider>();
            if (!coll) coll = zoneB.gameObject.AddComponent<BoxCollider>();
            coll.size = new Vector3(1f, 0.1f, 1f);
        }

        // 6. Setup the Interactive Camera
        if (Camera.main != null)
        {
            InteractiveCamera ic = Camera.main.gameObject.AddComponent<InteractiveCamera>();
            ic.target = armBase;
            
            // Move camera to a nice starting point
            Camera.main.transform.position = armBase.position + new Vector3(0, 3, -5);
            Camera.main.transform.LookAt(armBase.position + Vector3.up);
        }

        // Cleanup training utilities
        foreach(var oldScript in environment.GetComponentsInChildren<DataCollector>()) Object.DestroyImmediate(oldScript);

        string scenePath = "Assets/Scenes/RobotInteractiveDemo.unity";
        Directory.CreateDirectory("Assets/Scenes");
        UnityEditor.SceneManagement.EditorSceneManager.SaveScene(newScene, scenePath);
        
        Debug.Log("Interactive Demo Scene generated successfully at " + scenePath);
    }
}
