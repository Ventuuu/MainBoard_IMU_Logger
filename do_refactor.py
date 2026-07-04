import os
import re

main_file = r'd:/Polimi/2nd Sem/SWP_Github/MainBoard_IMU_Logger/Core/Src/main.c'

with open(main_file, 'r', encoding='utf-8') as f:
    content = f.read()

# We need to find specific functions and move them.
# A helper to extract a function by name
def extract_func(name, text):
    # Matches `static void name(...) { ... }` or similar
    pattern = r'((?:static\s+)?(?:void|int|uint8_t|uint16_t|uint32_t|LogStatus)\s+' + name + r'\s*\([^)]*\)\s*\{)'
    match = re.search(pattern, text)
    if not match:
        return None, text
    
    start_idx = match.start()
    
    # Simple brace counting
    brace_count = 0
    in_string = False
    in_char = False
    in_comment = False
    i = start_idx
    
    # move to first brace
    while text[i] != '{':
        i += 1
    
    while i < len(text):
        c = text[i]
        if in_comment:
            if c == '*' and i + 1 < len(text) and text[i+1] == '/':
                in_comment = False
                i += 1
        elif in_string:
            if c == '\\':
                i += 1
            elif c == '"':
                in_string = False
        elif in_char:
            if c == '\\':
                i += 1
            elif c == "'":
                in_char = False
        else:
            if c == '/' and i + 1 < len(text) and text[i+1] == '*':
                in_comment = True
                i += 1
            elif c == '/' and i + 1 < len(text) and text[i+1] == '/':
                # single line comment, skip to newline
                while i < len(text) and text[i] != '\n':
                    i += 1
            elif c == '"':
                in_string = True
            elif c == "'":
                in_char = True
            elif c == '{':
                brace_count += 1
            elif c == '}':
                brace_count -= 1
                if brace_count == 0:
                    break
        i += 1
        
    end_idx = i + 1
    func_text = text[start_idx:end_idx]
    new_text = text[:start_idx] + text[end_idx:]
    return func_text, new_text

# System ops functions
funcs_sys = ["SmartWearable_FactoryEraseNand"]
sys_code = ""

for f in funcs_sys:
    ftxt, content = extract_func(f, content)
    if ftxt:
        sys_code += ftxt + "\n\n"

# UI ops
funcs_ui = ["UpdateStateLed", "UserButton_Process", "UserButton_HandleShortPress"]
ui_code = ""
for f in funcs_ui:
    ftxt, content = extract_func(f, content)
    if ftxt:
        ui_code += ftxt + "\n\n"

# Sensor ops
funcs_sens = ["LiveMode_Start", "LiveMode_Stop", "SensorSuperframe_Init", "SensorSuperframe_Process",
              "SensorPhase_EnterEnvStart", "SensorPhase_EnterImuRun", "SensorPhase_EnterEnvEnd",
              "SensorPhase_StopImuRun", "SensorPhase_StartMicWindow", "SensorPhase_StopMicWindow",
              "SensorPhase_RequestLightMeasurement", "SensorPhase_IsEnvMetricsReady", "SensorPhase_TrySendStepBle"]
sens_code = ""
for f in funcs_sens:
    ftxt, content = extract_func(f, content)
    if ftxt:
        sens_code += ftxt + "\n\n"

# Rewrite main.c
with open(main_file, 'w', encoding='utf-8') as f:
    f.write(content)

base_dir = r'd:/Polimi/2nd Sem/SWP_Github/MainBoard_IMU_Logger/Core/Src'

def append_to_file(filename, text):
    if text:
        with open(os.path.join(base_dir, filename), 'a', encoding='utf-8') as f:
            f.write(text)

append_to_file('app_system_ops.c', sys_code)
append_to_file('app_state_ui.c', ui_code)
append_to_file('app_sensor_workflow.c', sens_code)

print("Extraction 1 done")
