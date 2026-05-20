import re
file_path = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Scripts\Demo\InteractiveRobotAgent.cs'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# find FixedUpdate
replacement = '''
    private void FixedUpdate()
    {
        if (isInteractiveMode)
        {
            if (targetZoneA) boxStartPosition = targetZoneA.position + Vector3.up * 0.5f;
            if (targetZoneB) targetPosition = targetZoneB.position + Vector3.up * 0.5f;
        }
        if (episodeEnding || movableBox == null) return;
'''

content = content.replace('    private void FixedUpdate()\n    {\n        if (episodeEnding || movableBox == null) return;', replacement)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)
