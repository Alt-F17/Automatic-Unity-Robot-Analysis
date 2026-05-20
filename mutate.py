import re

file_path = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Scripts\Demo\InteractiveRobotAgent.cs'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Replace class name
content = content.replace('public class RobotAgent : Agent', 'public class InteractiveRobotAgent : Agent')
content = content.replace('RobotAgent]', 'InteractiveRobotAgent]')

# Add interactive flag and coroutine
interactive_injection = '''
    [Header("Interactive Mode")]
    public bool isInteractiveMode = true;
    private bool isResetting = false;

    private void HandleEpisodeEnd()
    {
        if (isInteractiveMode)
        {
            if (!isResetting)
            {
                StartCoroutine(SmoothResetCoroutine());
            }
        }
        else
        {
            EndEpisode();
        }
    }

    private System.Collections.IEnumerator SmoothResetCoroutine()
    {
        isResetting = true;
        episodeEnding = true;

        // Briefly wait to show success
        yield return new WaitForSeconds(1.5f);
        
        EndEpisode(); 
        
        isResetting = false;
    }
'''

content = content.replace('public class InteractiveRobotAgent : Agent\n{', 'public class InteractiveRobotAgent : Agent\n{\n' + interactive_injection)

# Replace EndEpisode() with HandleEpisodeEnd()
content = content.replace('EndEpisode();', 'HandleEpisodeEnd();')

# Fix the two we just intentionally added in the injection so they don't loop
content = content.replace('else\n        {\n            HandleEpisodeEnd();', 'else\n        {\n            EndEpisode();')
content = content.replace('yield return new WaitForSeconds(1.5f);\n        \n        HandleEpisodeEnd();', 'yield return new WaitForSeconds(1.5f);\n        \n        EndEpisode();')

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)
print('Done modifying InteractiveRobotAgent.cs')
