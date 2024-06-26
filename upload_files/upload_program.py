import shutil
from rtm_funcs import *
from pathlib import Path

# For users who are more comfortable uploading directly from VS Code
# Run this script to upload the motor configuration to RTM test stand

# ENSURE RTM PROJECT RESIDES IN "C:/RTM/upload_files"
# IF NOT MOVE IT OR CHANGE THE PATH FOR project_dir
project_dir = Path("C:/RTM")

desktop = Path(os.path.join(os.environ["USERPROFILE"], "Desktop"))
onedrive_desktop = Path(os.path.join(os.environ["USERPROFILE"], "OneDrive", "Desktop"))
if onedrive_desktop.exists():
    desktop = onedrive_desktop
config_xlsx = project_dir / "upload_files" / "RTM_config.xlsx"
new_config_xlsx = desktop / "RTM_config.xlsx"
if not new_config_xlsx.exists():
    shutil.copy(config_xlsx, desktop)
    print("'RTM_config.xlsx' was not present on the desktop, so a generic one has been created.")
    print("Please edit it to reflect the desired test profile and re-run the 'RTM Upload' program.")


config_xlsx = desktop / "RTM_config.xlsx"
# Load configurations and steps from Excel
steps, config = load_config_from_excel(config_xlsx)

# Generate the motor_config.h file
generate_motor_config(steps, config)

# Compile and upload the project
compile_and_upload(project_dir)