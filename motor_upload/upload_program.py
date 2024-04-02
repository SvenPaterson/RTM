# Specify the project directory and the environment name
from rtm_funcs import *
from pathlib import Path

config_xlsx = Path.cwd() / 'motor_upload' / 'RTM_config.xlsx'
project_dir = Path.cwd()

environment = 'motor'

# Load configurations and steps from Excel
steps, config = load_config_from_excel(config_xlsx)

# Generate the motor_config.h file
generate_motor_config(steps, config)

# Compile and upload the project
compile_and_upload(project_dir, environment)


