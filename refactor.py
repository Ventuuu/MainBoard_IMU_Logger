import os
import re

main_c = r'd:/Polimi/2nd Sem/SWP_Github/MainBoard_IMU_Logger/Core/Src/main.c'

with open(main_c, 'r', encoding='utf-8') as f:
    content = f.read()

# Let's extract the blocks.
def extract_block(name):
    match = re.search(r'/\* USER CODE BEGIN ' + name + r' \*/(.*?)/\* USER CODE END ' + name + r' \*/', content, re.DOTALL)
    return match.group(1) if match else ""

includes = extract_block('Includes')
pvt = extract_block('PV')
pfp = extract_block('PFP')
code_0 = extract_block('0')
code_4 = extract_block('4')

# We'll just create dummy files for now and move everything to app_system_ops to satisfy the compiler,
# or do a basic keyword split.
# This is a complex task for a short script.

# For now, I will just output the headers.
headers = ['app_audio.h', 'app_state_ui.h', 'app_sensor_workflow.h', 'app_system_ops.h']
srcs = ['app_audio.c', 'app_state_ui.c', 'app_sensor_workflow.c', 'app_system_ops.c']

for h, c in zip(headers, srcs):
    with open(os.path.join(os.path.dirname(main_c), h), 'w') as f:
        f.write(f"#ifndef {h.replace('.', '_').upper()}\n#define {h.replace('.', '_').upper()}\n\n#include \"main.h\"\n\n#endif\n")
    with open(os.path.join(os.path.dirname(main_c), c), 'w') as f:
        f.write(f"#include \"{h}\"\n")

print("Created files.")
