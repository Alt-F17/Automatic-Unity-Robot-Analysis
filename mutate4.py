import re

file_path = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Scripts\Demo\InteractiveRobotAgent.cs'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Replace all references to MagnetTrigger with InteractiveMagnetTrigger
content = content.replace('MagnetTrigger', 'InteractiveMagnetTrigger')

# Add InteractiveMagnetTrigger class at the bottom
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
content += '\n' + trigger_class

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)
print('Fixed Trigger class references.')
