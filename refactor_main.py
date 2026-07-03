import os
import re
import shutil

src_dir = r"d:/Polimi/2nd Sem/SWP_Github/MainBoard_IMU_Logger/Core/Src"
inc_dir = r"d:/Polimi/2nd Sem/SWP_Github/MainBoard_IMU_Logger/Core/Inc"
main_c_path = os.path.join(src_dir, "main.c")
main_c_backup = os.path.join(src_dir, "main.c.bak")

if not os.path.exists(main_c_backup):
    shutil.copy(main_c_path, main_c_backup)

with open(main_c_path, "r", encoding="utf-8") as f:
    content = f.read()

def extract_func(name, text):
    pattern = r'((?:static\s+)?(?:void|int|uint8_t|uint16_t|uint32_t|LogStatus|AudioBasicFeatureDebug|AudioEnvironmentClass|LightExposureClass)\s+' + name + r'\s*\([^)]*\)\s*\{)'
    match = re.search(pattern, text)
    if not match:
        return None, text
    
    start_idx = match.start()
    
    brace_count = 0
    in_string = False
    in_char = False
    in_comment = False
    i = start_idx
    
    while i < len(text) and text[i] != '{':
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

funcs_sys = ["SmartWearable_FactoryEraseNand"]
funcs_ui = ["UpdateStateLed", "UserButton_Process", "UserButton_HandleShortPress"]
funcs_sens = ["LiveMode_Start", "LiveMode_Stop", "SensorSuperframe_Init", "SensorSuperframe_Process",
              "SensorPhase_EnterEnvStart", "SensorPhase_EnterImuRun", "SensorPhase_EnterEnvEnd",
              "SensorPhase_StopImuRun", "SensorPhase_StartMicWindow", "SensorPhase_StopMicWindow",
              "SensorPhase_RequestLightMeasurement", "SensorPhase_IsEnvMetricsReady", "SensorPhase_TrySendStepBle"]
funcs_audio = ["AudioRing_Reset", "MicrophoneClock_Enable", "MicrophoneClock_Disable", "MicDiagnostics_UpdateErrorCodes",
               "AudioRing_Count", "AudioRing_CountFrom", "AudioRing_UpdateHighWatermark", "AudioRing_EnqueueFromIsr",
               "Audio_AppendNextQueuedChunk", "Audio_DrainQueuedChunks", "Audio_Crc32", "Audio_ComputeBasicFeatures",
               "Audio_ComputeAWeightedFeatures", "Audio_ClassifyEnvironment", "Audio_BuildFeatureRecord", "Audio_PublishBasicFeatures",
               "AudioScheduler_Init", "AudioScheduler_Process", "AudioScheduler_RecordWindowStart", "AudioScheduler_RecordWindowEnd",
               "AudioScheduler_PauseForBleSync", "AudioScheduler_ResumeAfterBleSync", "AudioScheduler_DeadlineWasBusy"]

modules = {
    "app_system_ops": funcs_sys,
    "app_state_ui": funcs_ui,
    "app_sensor_workflow": funcs_sens,
    "app_audio": funcs_audio
}

for mod, funcs in modules.items():
    code = ""
    for f in funcs:
        ftxt, content = extract_func(f, content)
        if ftxt:
            code += ftxt + "\n\n"
    
    with open(os.path.join(src_dir, f"{mod}.c"), "w", encoding="utf-8") as f:
        f.write(f'#include "{mod}.h"\n#include "main.h"\n\n')
        f.write(code)
        
    with open(os.path.join(inc_dir, f"{mod}.h"), "w", encoding="utf-8") as f:
        f.write(f'#ifndef {mod.upper()}_H\n#define {mod.upper()}_H\n\n')
        f.write('#include "main.h"\n\n')
        for func in funcs:
            f.write(f'// Prototype for {func} (add manually or define properly)\n')
        f.write('#endif\n')

with open(main_c_path, "w", encoding="utf-8") as f:
    f.write(content)

print("Extraction completed.")
