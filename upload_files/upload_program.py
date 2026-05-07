import argparse
import os
import shutil
from pathlib import Path

from rtm_funcs import compile_and_upload, generate_motor_config, load_config_from_excel

def resolve_project_dir() -> Path:
    user_profile = Path(os.environ["USERPROFILE"])
    if Path("C:/RTM").exists():
        return Path("C:/RTM")
    return user_profile / "RTM"


def resolve_config_path(project_dir: Path, args: argparse.Namespace) -> Path:
    local_config = project_dir / "upload_files" / "RTM_continuous_config.xlsx"
    desktop = Path(os.environ["USERPROFILE"]) / "Desktop"
    desktop_config = desktop / "RTM_continuous_config.xlsx"

    if args.config_path:
        return Path(args.config_path)

    if args.config_source == "desktop":
        if not desktop_config.exists():
            shutil.copy(local_config, desktop)
            print("Desktop config missing. Copied template to Desktop.")
            print("Edit it and re-run the upload script.")
        return desktop_config

    return local_config


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate motor_config.h from Excel and upload motor firmware."
    )
    parser.add_argument(
        "--config-source",
        choices=["local", "desktop"],
        default="local",
        help="Choose config source when --config-path is not provided.",
    )
    parser.add_argument(
        "--config-path",
        default=None,
        help="Explicit path to RTM_continuous_config.xlsx.",
    )
    args = parser.parse_args()

    project_dir = resolve_project_dir()
    config_xlsx = resolve_config_path(project_dir, args)

    if not config_xlsx.exists():
        raise FileNotFoundError(f"Config file not found: {config_xlsx}")

    print(f"Project directory: {project_dir}")
    print(f"Using config file: {config_xlsx}")

    # Load configurations and steps from Excel
    steps, config = load_config_from_excel(config_xlsx)

    # Generate the motor_config.h file in this project directory
    generate_motor_config(steps, config, project_dir)

    # Compile and upload the project
    compile_and_upload(project_dir)


if __name__ == "__main__":
    main()