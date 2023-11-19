import os
import re

def find_latest_model(directory, prefix): 
    files = os.listdir(directory)
    relevant_files = [f for f in files if f.startswith(prefix)]
    if not relevant_files:
        return None,0
    numbers = []
    for file in relevant_files:
        match = re.search(r'(\d+)\.zip$', file)
        if match:
            numbers.append(int(match.group(1)))
    if not numbers:
        return None,0
    max_num = max(numbers)
    return f"{prefix}_{max_num}.zip",max_num
