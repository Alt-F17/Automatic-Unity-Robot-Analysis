using UnityEngine;

/// <summary>
/// Spawns multiple training areas in a grid for parallel training
/// </summary>
public class TrainingAreaSpawner : MonoBehaviour
{
    [Header("Spawning Settings")]
    [SerializeField] private GameObject trainingAreaPrefab;
    [SerializeField] private int numberOfAreas = 100;
    [SerializeField] private float spacing = 15f;  // Space between areas
    
    [Header("Grid Layout")]
    [SerializeField] private int areasPerRow = 10;  // 10x10 grid for 100 areas
    
    void Start()
    {
        SpawnTrainingAreas();
    }
    
    void SpawnTrainingAreas()
    {
        if (trainingAreaPrefab == null)
        {
            Debug.LogError("[Spawner] No prefab assigned!");
            return;
        }
        
        Debug.Log($"[Spawner] Creating {numberOfAreas} training areas...");
        
        for (int i = 0; i < numberOfAreas; i++)
        {
            // Calculate grid position
            int row = i / areasPerRow;
            int col = i % areasPerRow;
            
            Vector3 position = new Vector3(
                col * spacing,
                0,
                row * spacing
            );
            
            // Spawn the area
            GameObject area = Instantiate(trainingAreaPrefab, position, Quaternion.identity);
            area.name = $"TrainingArea_{i}";
            area.transform.parent = transform;  // Organize under this object
        }
        
        Debug.Log($"[Spawner] Successfully spawned {numberOfAreas} areas!");
    }
}