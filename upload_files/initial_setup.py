import shutil, os
from pathlib import Path

desktop = Path(os.path.join(os.environ["USERPROFILE"], "Desktop"))
project_dir = Path("C:/RTM/upload_files")
config_xlsx = project_dir / "RTM_config.xlsx"
new_config_xlsx = desktop / "RTM_config.xlsx"
upload_shrtcut = project_dir / "RTM Upload.lnk"
new_upload_shrtcut = desktop / "RTM Upload.lnk"


if not new_config_xlsx.exists():
    shutil.copy(config_xlsx, desktop)
if not new_upload_shrtcut.exists():
    shutil.copy(upload_shrtcut, desktop)