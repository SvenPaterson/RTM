import shutil
import os
from pathlib import Path

# For new users who want to run the RTM Upload software from the desktop
# Run this setup once to load the correct files onto the desktop

try:
    desktop = Path(os.path.join(os.environ["USERPROFILE"], "Desktop"))
    onedrive_desktop = Path(os.path.join(os.environ["USERPROFILE"], "OneDrive", "Desktop"))
    if onedrive_desktop.exists():
        desktop = onedrive_desktop
    project_dir = Path("C:/RTM/upload_files")
    config_xlsx = project_dir / "RTM_config.xlsx"
    new_config_xlsx = desktop / "RTM_config.xlsx"
    upload_shrtcut = project_dir / "RTM Upload.lnk"
    new_upload_shrtcut = desktop / "RTM Upload.lnk"

    print(f"Attempting to copy: {config_xlsx} to {new_config_xlsx}")
    if not new_config_xlsx.exists():
        shutil.copy(config_xlsx, desktop)
    else:
        print(f"The file {new_config_xlsx} already exists on the desktop.")

    print(f"Attempting to copy: {upload_shrtcut} to {new_upload_shrtcut}")
    if not new_upload_shrtcut.exists():
        shutil.copy(upload_shrtcut, desktop)
    else:
        print(f"The shortcut {new_upload_shrtcut} already exists on the desktop.")

except Exception as e:
    print(f"An error occurred: {e}")
