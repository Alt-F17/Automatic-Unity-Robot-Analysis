import re

prefab_path = r'd:\Unity\Projects\Automatic-Unity-Robot-Analysis\Assets\Prefabs\TrainingArea_0.prefab'

with open(prefab_path, 'r', encoding='utf-8') as f:
    text = f.read()

objects = {}
for i, block in enumerate(text.split('---')):
    m = re.search(r'm_Name: (.*)', block)
    if m:
        game_object_id = re.search(r'!u!1 &(\d+)', block)
        if game_object_id:
            objects[game_object_id.group(1)] = m.group(1).strip()

hierarchy = {}
transforms = {}

for i, block in enumerate(text.split('---')):
    if 'Transform:' in block:
        game_object_id = re.search(r'm_GameObject: \{fileID: (\d+)\}', block)
        transform_id = re.search(r'!u!4 &(\d+)', block)
        father_id = re.search(r'm_Father: \{fileID: (\d+)\}', block)
        
        if game_object_id and transform_id:
            tid = transform_id.group(1)
            pos_x = re.search(r'm_LocalPosition:.*?x: ([\d\.\-]+)', block, re.DOTALL)
            pos_y = re.search(r'm_LocalPosition:.*?y: ([\d\.\-]+)', block, re.DOTALL)
            pos_z = re.search(r'm_LocalPosition:.*?z: ([\d\.\-]+)', block, re.DOTALL)
            transforms[tid] = {
                'game_object': game_object_id.group(1),
                'pos': (float(pos_x.group(1)), float(pos_y.group(1)), float(pos_z.group(1))) if pos_x else (0,0,0),
                'father': father_id.group(1) if father_id else None
            }

for tid, data in transforms.items():
    if data['father'] in transforms:
        data['father_name'] = objects.get(transforms[data['father']]['game_object'], "Unknown")
    else:
        data['father_name'] = "Root"

for tid, data in transforms.items():
    name = objects.get(data['game_object'], "Unknown")
    if 'Shoulder' in name or 'Elbow' in name or 'Magnet' in name or 'Arm' in name or 'Base' in name:
        print(f"{name} -> Parent: {data['father_name']} | LocalPos: {data['pos']}")

