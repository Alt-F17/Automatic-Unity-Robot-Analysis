using UnityEngine;

public class TrainingAreaSpawner : MonoBehaviour
{
    [Header("Spawning Settings")]
    [SerializeField] private GameObject trainingAreaPrefab;
    [SerializeField] private int numberOfAreas = 100;
    [SerializeField] private float spacing = 15f;
    
    [Header("Grid Layout")]
    [SerializeField] private int areasPerRow = 10;
    
    void Start()
    {
        // Add PerformanceTracker if not present
        if (FindObjectOfType<PerformanceTracker>() == null)
        {
            GameObject tracker = new GameObject("PerformanceTracker");
            tracker.AddComponent<PerformanceTracker>();
        }
        
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
            int row = i / areasPerRow;
            int col = i % areasPerRow;
            
            Vector3 position = new Vector3(
                col * spacing,
                0,
                row * spacing
            );
            
            GameObject area = Instantiate(trainingAreaPrefab, position, Quaternion.identity);
            area.name = $"TrainingArea_{i}";
            area.transform.parent = transform;
        }
        
        Debug.Log($"[Spawner] Successfully spawned {numberOfAreas} areas!");
    }
}