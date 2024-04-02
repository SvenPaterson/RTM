import subprocess
import pandas as pd

def load_config_from_excel(file_path):
    # Load system configuration
    config_df = pd.read_excel(file_path, sheet_name='Config', usecols="A:B", header=0, engine='openpyxl').dropna()
    config = {row['Setting']: row['Value'] for _, row in config_df.iterrows()}
    
    # Load steps, skipping the first row of data (units)
    steps_df = pd.read_excel(file_path, sheet_name='Steps', usecols="A:F", header=0, skiprows=[1], engine='openpyxl')
    steps = []
    for _, row in steps_df.iterrows():
        steps.append({
            "turnOnHeat": bool(row.iloc[1]),
            "target_speed": row.iloc[2],
            "is_CCW": row.iloc[3],
            "accel": row.iloc[4],
            "time": row.iloc[5]
        })
    
    return steps, config

def generate_motor_config(steps, config):
    with open("./include/motor_config.h", "w") as file:
        file.write("#ifndef motor_config_h\n")
        file.write("#define motor_config_h\n\n")
        file.write("#include <cstdint>\n\n")
        
        file.write(f"const bool isHighSpeedGearBox = {'true' if config['isHighSpeedGearBox'] else 'false'};\n")
        file.write(f"const uint16_t SPR = {config['SPR_3'] if config['isHighSpeedGearBox'] else config['SPR']};\n\n")
        
        file.write("struct Step {\n")
        file.write("    bool turnOnHeat;\n")
        file.write("    double target_speed;\n")
        file.write("    bool is_CCW;\n")
        file.write("    double accel;\n")
        file.write("    uint32_t time;\n")
        file.write("};\n\n")
        
        file.write("const Step steps[] = {\n")
        for step in steps:
            file.write(f"    {{{'true' if step['turnOnHeat'] else 'false'}, {step['target_speed']}, {'true' if step['is_CCW'] else 'false'}, {step['accel']}, {step['time']}}},\n")
        
        file.write("};\n\n")
        file.write("#endif\n")


def compile_and_upload(project_directory, environment):
    compile_command = ["C:/Users/stephen/.platformio/penv/Scripts/platformio.exe", 
                       "run", "--target", "upload", "--environment",
                       environment, "-d", project_directory]

    try:
        # Compile and upload the project
        print("Compiling and uploading to the board...")
        result = subprocess.run(compile_command, check=True, capture_output=True, text=True)
        print(result.stdout)
        
    except subprocess.CalledProcessError as e:
        print(f"Error during compilation/upload: {e.stderr}")