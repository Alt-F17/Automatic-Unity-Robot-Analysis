import re

src = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Scripts\Agent\RobotAgent.cs'
dst = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Scripts\Demo\InteractiveRobotAgent.cs'

with open(src, 'r', encoding='utf-8') as f:
    text = f.read()

text = text.replace('public class RobotAgent : Agent', 'public class InteractiveRobotAgent : Agent')
text = text.replace('RobotAgent]', 'InteractiveRobotAgent]')

# Strip MagnetTrigger from the bottom of the file
idx = text.find('public class MagnetTrigger : MonoBehaviour')
if idx != -1:
    text = text[:idx]

# Inject interactive coroutine at the top of the class
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
text = text.replace('public class InteractiveRobotAgent : Agent\n{', 'public class InteractiveRobotAgent : Agent\n{\n' + interactive_injection)

# Replace EndEpisode() with HandleEpisodeEnd() globally inside the class methods
text = text.replace('EndEpisode();', 'HandleEpisodeEnd();')

# Re-fix the intentionally added EndEpisode() in the injection
text = text.replace('else\n        {\n            HandleEpisodeEnd();', 'else\n        {\n            EndEpisode();')
text = text.replace('yield return new WaitForSeconds(1.5f);\n        \n        HandleEpisodeEnd();', 'yield return new WaitForSeconds(1.5f);\n        \n        EndEpisode();')

# Override fixed update for dynamic target positioning
replacement_update = '''
    private void FixedUpdate()
    {
        if (isInteractiveMode)
        {
            if (targetZoneA) boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
            if (targetZoneB) targetPosition = targetZoneB.position + Vector3.up * 0.5f;
        }
        if (episodeEnding || movableBox == null) return;
'''
text = text.replace('    private void FixedUpdate()\n    {\n        if (episodeEnding || movableBox == null) return;', replacement_update)

# Fix references for DataCollector warning (ignoring for now as it's a warning)
# Fix PerformanceTracker references
text = text.replace('PerformanceTracker.Instance.RecordEpisode(this', 'PerformanceTracker.Instance.RecordEpisode((RobotAgent)(Agent)this')
text = text.replace('PerformanceTracker.Instance.IsBestPerformer(this)', 'PerformanceTracker.Instance.IsBestPerformer((RobotAgent)(Agent)this)')

# Now, append the InteractiveMagnetTrigger at the end to replace the one we stripped
trigger_class = '''
public class InteractiveMagnetTrigger : MonoBehaviour
{
    private InteractiveRobotAgent agent;

    public void Initialize(InteractiveRobotAgent parentAgent)
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

    void OnTriggerStay(Collider other)
    {
        if (agent != null)
        {
            agent.OnMagnetTriggerEnter(other);
        }
    }
}
'''
text += '\n' + trigger_class

# Lastly, fix the MagnetTrigger instantiation inside the agent:
text = text.replace('MagnetTrigger trigger = magnet.GetComponent<MagnetTrigger>();', 'InteractiveMagnetTrigger trigger = magnet.GetComponent<InteractiveMagnetTrigger>();')
text = text.replace('trigger = magnet.gameObject.AddComponent<MagnetTrigger>();', 'trigger = magnet.gameObject.AddComponent<InteractiveMagnetTrigger>();')

with open(dst, 'w', encoding='utf-8') as f:
    f.write(text)
print('InteractiveRobotAgent cloned and correctly mutated.')
