prefab_path = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Prefabs\TrainingArea_0.prefab'

with open(prefab_path, 'r', encoding='utf-8') as f:
    text = f.read()

import re

blocks = text.split('---')

name_to_id = {}
for b in blocks:
    if 'GameObject:' in b:
        m = re.search(r'!u!1 &(\d+)', b)
        if m:
            obj_id = m.group(1).strip()
            name_m = re.search(r'm_Name: (.*)', b)
            if name_m:
                name_to_id[obj_id] = name_m.group(1).strip()

id_to_transform = {}
for b in blocks:
    if 'Transform:' in b:
        # get game object ID
        obj_m = re.search(r'm_GameObject: {fileID: (.*?)}', b)
        if obj_m:
            obj_id = obj_m.group(1).strip()
            pos_m = re.search(r'm_LocalPosition:\s+x: ([\d\.\-]+)\s+y: ([\d\.\-]+)\s+z: ([\d\.\-]+)', b)
            if pos_m:
                id_to_transform[obj_id] = (float(pos_m.group(1)), float(pos_m.group(2)), float(pos_m.group(3)))

for obj_id, name in name_to_id.items():
    if 'Shoulder' in name or 'Elbow' in name or 'Base' in name or 'Magnet' in name or 'Arm' in name or 'Effector' in name:
        if obj_id in id_to_transform:
            print(f"{name}: {id_to_transform[obj_id]}")

