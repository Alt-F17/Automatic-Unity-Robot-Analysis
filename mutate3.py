import re

file_path = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Scripts\Demo\InteractiveRobotAgent.cs'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# remove public class MagnetTrigger and everything after it
index = content.find('public class MagnetTrigger : MonoBehaviour')
if index != -1:
    content = content[:index]

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)
print('MagnetTrigger removed.')
