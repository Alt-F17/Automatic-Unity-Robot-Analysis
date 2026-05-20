import re

prefab_path = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Prefabs\TrainingArea_0.prefab'

with open(prefab_path, 'r', encoding='utf-8') as f:
    text = f.read()

objects = {}
for i, block in enumerate(text.split('---')):
    m = re.search(r'm_Name: (.*)', block)
    if m:
        # find the associated Transform
        game_object_id = re.search(r'!u!1 &(\d+)', block)
        if game_object_id:
            objects[game_object_id.group(1)] = m.group(1).strip()

transforms = {}
for i, block in enumerate(text.split('---')):
    if 'Transform:' in block:
        game_object_id = re.search(r'm_GameObject: \{fileID: (\d+)\}', block)
        if game_object_id:
            pos_x = re.search(r'm_LocalPosition:.*?x: ([\d\.\-]+)', block, re.DOTALL)
            pos_y = re.search(r'm_LocalPosition:.*?y: ([\d\.\-]+)', block, re.DOTALL)
            pos_z = re.search(r'm_LocalPosition:.*?z: ([\d\.\-]+)', block, re.DOTALL)
            if pos_x and pos_y and pos_z:
                transforms[game_object_id.group(1)] = (float(pos_x.group(1)), float(pos_y.group(1)), float(pos_z.group(1)))

for obj_id, name in objects.items():
    if 'Shoulder' in name or 'Elbow' in name or 'Magnet' in name or 'Arm' in name or 'Base' in name:
        if obj_id in transforms:
            print(f"{name}: {transforms[obj_id]}")
