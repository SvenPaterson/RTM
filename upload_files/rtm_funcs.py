import subprocess, os
from pathlib import Path
import pandas as pd


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y", "on"}
    return bool(value)

def load_config_from_excel(file_path):
    # Load system configuration
    config_df = pd.read_excel(file_path, sheet_name='Config', usecols="A:B", header=0, engine='openpyxl').dropna()
    config = {row['Setting']: row['Value'] for _, row in config_df.iterrows()}
    
    # Load steps, skipping the first row of data (units)
    steps_df = pd.read_excel(file_path, sheet_name='Steps', usecols="A:F", header=0, skiprows=[1], engine='openpyxl')
    steps = []
    for _, row in steps_df.iterrows():
        # Only parse rows that are explicitly populated by the user.
        # Column A is the step identifier/order field.
        if pd.isna(row.iloc[0]):
            continue

        # Skip partial/invalid rows to avoid emitting NaN values in C++.
        if pd.isna(row.iloc[1]) or pd.isna(row.iloc[2]) or pd.isna(row.iloc[3]) or pd.isna(row.iloc[4]) or pd.isna(row.iloc[5]):
            continue

        steps.append({
            "turnOnHeat": _parse_bool(row.iloc[1]),
            "target_speed": row.iloc[2],
            "is_CCW": row.iloc[3],
            "accel": row.iloc[4],
            "time": row.iloc[5]
        })
    
    return steps, config

def generate_motor_config(steps, config, project_directory):
    required_keys = ["SPR", "SPR_3", "isHighSpeedGearBox"]
    missing_keys = [k for k in required_keys if k not in config]
    if missing_keys:
        raise KeyError(f"Missing required Config keys: {', '.join(missing_keys)}")

    # Optional override: if customSPR is present and positive, use it directly
    # instead of the gearbox-based selection. Useful when a specific motor/drive
    # is configured for a non-standard PPR and you can't reach the drive to
    # change it.
    custom_spr_raw = config.get("customSPR")
    has_custom = (
        custom_spr_raw not in (None, "")
        and not (isinstance(custom_spr_raw, float) and pd.isna(custom_spr_raw))
    )
    if has_custom:
        try:
            spr_value = int(float(custom_spr_raw))
        except (TypeError, ValueError):
            raise ValueError(f"customSPR must be a number, got: {custom_spr_raw!r}")
        if spr_value <= 0:
            raise ValueError(f"customSPR must be positive, got: {spr_value}")
        print(f"Using customSPR override: {spr_value}")
    else:
        is_high_speed = _parse_bool(config["isHighSpeedGearBox"])
        spr_value = int(config["SPR_3"] if is_high_speed else config["SPR"])

    output_path = Path(project_directory) / "include" / "motor_config.h"
    with open(output_path, "w") as file:
        file.write("#ifndef motor_config_h\n")
        file.write("#define motor_config_h\n\n")
        file.write("#include <cstdint>\n\n")
        
        file.write(f"const uint16_t SPR = {spr_value};\n\n")
        
        file.write("struct Step {\n")
        file.write("    bool turnOnHeat;\n")
        file.write("    double target_speed;\n")
        file.write("    bool is_CCW;\n")
        file.write("    double accel;\n")
        file.write("    double time;\n")
        file.write("};\n\n")
        
        file.write("const Step steps[] = {\n")
        for step in steps:
            file.write(f"    {{{'true' if step['turnOnHeat'] else 'false'}, {step['target_speed']}, {'true' if step['is_CCW'] == 'CCW' else 'false'}, {step['accel']}, {step['time']}}},\n")
        
        file.write("};\n\n")
        file.write("#endif\n")

    print(f"Generated motor config: {output_path}")


def compile_and_upload(project_directory):
    user_profile = os.getenv('USERPROFILE')
    platformio_path = os.path.join(user_profile, '.platformio', 'penv', 'Scripts', 'platformio.exe')
    compile_command = [platformio_path, "run", "--target", "upload", "--environment", 'motor', "-d", project_directory]

    try:
        # Compile and upload the project
        print("Compiling and uploading to the board...")
        result = subprocess.run(compile_command, check=True, capture_output=True, text=True)
        print(result.stdout)
        
    except subprocess.CalledProcessError as e:
        print(f"Error during compilation/upload: {e.stderr}")