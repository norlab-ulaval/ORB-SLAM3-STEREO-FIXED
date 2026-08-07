#!/usr/bin/env python3
import argparse
import os

def update_bash(file_path, dataset, output, trajectory, yaml_file):
    if not os.path.exists(file_path):
        return
    with open(file_path, 'r') as f:
        lines = f.readlines()

    out_lines = []
    for line in lines:
        if line.strip().startswith('DATASET_PATH=') and dataset:
            out_lines.append(f'DATASET_PATH="{dataset}"\n')
        elif line.strip().startswith('OUTPUT_PATH=') and output:
            out_lines.append(f'OUTPUT_PATH="{output}"\n')
        elif line.strip().startswith('TRAJECTORY_NAME=') and trajectory:
            out_lines.append(f'TRAJECTORY_NAME="{trajectory}"\n')
        else:
            # Point to the scripted yaml instead of fomo.yaml
            if yaml_file:
                line = line.replace('fomo.yaml', yaml_file)
            out_lines.append(line)

    with open(file_path, 'w') as f:
        f.writelines(out_lines)

def env_flag(name, default=False):
    val = os.environ.get(name)
    if val is None:
        return default
    return val.strip().lower() in ('1', 'true', 'yes', 'on')

def update_yaml(file_path, load_atlas, save_atlas, realtime):
    if not os.path.exists(file_path):
        print(f"Warning: {file_path} not found.")
        return
    with open(file_path, 'r') as f:
        lines = f.readlines()

    # The lightweight feature settings below only make sense when we are both
    # localizing against an existing atlas and running in real time. Off the
    # clock we would rather keep the detail: it matters a lot when the scene
    # has changed since the map was built.
    fast = bool(load_atlas) and realtime

    out_lines = []
    load_atlas_set = False
    loc_mode_set = False
    save_atlas_set = False

    for line in lines:
        if 'System.LoadAtlasFromFile:' in line:
            if load_atlas and not load_atlas_set:
                out_lines.append(f'System.LoadAtlasFromFile: "{load_atlas}"\n')
                load_atlas_set = True
            elif not load_atlas:
                if not line.strip().startswith('#'):
                    out_lines.append('# ' + line.lstrip())
                else:
                    out_lines.append(line)
            else:
                if not line.strip().startswith('#'):
                    out_lines.append('# ' + line.lstrip())
                else:
                    out_lines.append(line)
            continue

        if 'System.LocalizationMode:' in line:
            if load_atlas and not loc_mode_set:
                out_lines.append(f'System.LocalizationMode: 1\n')
                loc_mode_set = True
            elif not load_atlas and not loc_mode_set:
                out_lines.append(f'System.LocalizationMode: 0\n')
                loc_mode_set = True
            else:
                if not line.strip().startswith('#'):
                    out_lines.append('# ' + line.lstrip())
                else:
                    out_lines.append(line)
            continue

        if 'Camera.imageScale:' in line:
            # Downscale images to save ORB-extraction time, so tracking can
            # keep up with the frame budget.
            scale = 1.0 if fast else 1.0
            out_lines.append(f'Camera.imageScale: {scale}\n')
            continue

        if 'ORBextractor.nFeatures:' in line:
            # Fewer features needed when we're only matching against an
            # existing map rather than building one.
            nfeatures = 4000 if fast else 4000
            out_lines.append(f'ORBextractor.nFeatures: {nfeatures}\n')
            continue

        if 'ORBextractor.nLevels:' in line:
            # A smaller image (Camera.imageScale above) can't support as many
            # pyramid levels before the finest level runs out of pixels for
            # the grid-based FAST detector, so shrink this alongside the scale.
            nlevels = 13 if fast else 13
            out_lines.append(f'ORBextractor.nLevels: {nlevels}\n')
            continue

        if 'System.SaveAtlasToFile:' in line:
            if save_atlas and not save_atlas_set:
                out_lines.append(f'System.SaveAtlasToFile: "{save_atlas}"\n')
                save_atlas_set = True
            elif not save_atlas:
                if not line.strip().startswith('#'):
                    out_lines.append('# ' + line.lstrip())
                else:
                    out_lines.append(line)
            else:
                if not line.strip().startswith('#'):
                    out_lines.append('# ' + line.lstrip())
                else:
                    out_lines.append(line)
            continue

        out_lines.append(line)

    # If the lines were completely missing, insert them
    if load_atlas and not load_atlas_set:
        out_lines.insert(3, f'System.LoadAtlasFromFile: "{load_atlas}"\n')
    if load_atlas and not loc_mode_set:
        out_lines.insert(4, f'System.LocalizationMode: 1\n')
    if save_atlas and not save_atlas_set:
        out_lines.insert(5, f'System.SaveAtlasToFile: "{save_atlas}"\n')

    with open(file_path, 'w') as f:
        f.writelines(out_lines)

def main():
    parser = argparse.ArgumentParser(description="Configure ORB-SLAM3 scripts and YAML parameters")
    parser.add_argument("--dataset", help="Set DATASET_PATH in bash scripts")
    parser.add_argument("--output", help="Set OUTPUT_PATH in bash scripts")
    parser.add_argument("--trajectory", help="Set TRAJECTORY_NAME in bash scripts")
    parser.add_argument("--load-atlas", help="Set System.LoadAtlasFromFile in YAML and auto-enable LocalizationMode. Pass empty string to disable.", default=None)
    parser.add_argument("--save-atlas", help="Set System.SaveAtlasToFile in YAML. Pass empty string to disable.", default=None)

    args = parser.parse_args()

    yaml_file = "fomo_scripted.yaml"
    workspace_dir = os.path.dirname(os.path.abspath(__file__))

    run_sh = os.path.join(workspace_dir, "run.sh")
    run_ros_sh = os.path.join(workspace_dir, "run_ros.sh")
    yaml_path = os.path.join(workspace_dir, yaml_file)

    if args.dataset or args.output or args.trajectory:
        update_bash(run_sh, args.dataset, args.output, args.trajectory, yaml_file)
        update_bash(run_ros_sh, args.dataset, args.output, args.trajectory, yaml_file)
        print("Updated bash scripts.")
    else:
        # Just update to use fomo_scripted.yaml
        update_bash(run_sh, None, None, None, yaml_file)
        update_bash(run_ros_sh, None, None, None, yaml_file)

    realtime = env_flag("ORB_REALTIME")

    if args.load_atlas is not None or args.save_atlas is not None:
        update_yaml(yaml_path, args.load_atlas, args.save_atlas, realtime)
        print(f"Updated YAML. LoadAtlas: {args.load_atlas or 'disabled'}, SaveAtlas: {args.save_atlas or 'disabled'}. LocalizationMode: {'true' if args.load_atlas else 'false'}. Realtime: {'true' if realtime else 'false'} (lightweight features: {'on' if args.load_atlas and realtime else 'off'})")

if __name__ == "__main__":
    main()
