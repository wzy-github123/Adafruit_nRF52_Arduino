import sys
import subprocess


def main(uf2_file: str, board_id: str, build_path: str, output_path: str, project_name: str):
    if board_id == "TRACKER_T1000_E_LORAWAN":
        try:
            command = [
                "python",
                uf2_file,
                "-f",
                "0xADA52840",
                "-c",
                "-o",
                f"{output_path}/{project_name}.uf2",
                f"{build_path}/{project_name}.hex",
            ]
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error executing uf2convert.py: {e}")
    else:
        print(
            f"Skipping UF2 conversion for board '{board_id}'"
        )


if __name__ == "__main__":
    if len(sys.argv) != 6:
        print(sys.argv)
        print(
            "Usage: python script.py <board_id> <expected_board> <build_path> <output_path> <project_name>"
        )
        sys.exit(1)

    uf2_file, board_id, build_path, output_path, project_name = sys.argv[1:]
    main(uf2_file, board_id, build_path, output_path, project_name)
