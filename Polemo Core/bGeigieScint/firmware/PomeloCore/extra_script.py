#!/usr/bin/env python3
"""
PlatformIO post-build script for PomeloCore
Converts firmware.bin to firmware.uf2 format for UF2 bootloader
"""

Import("env")
import subprocess
import os

def after_build(source, target, env):
    """
    Post-build action to convert .bin to .uf2 format
    This replicates the Atmel Studio post-build command:
    python "$(SolutionDir)\\uf2conv.py" "$(OutputDirectory)\\$(OutputFileName).bin" -o "$(SolutionDir)\\$(OutputFileName).uf2"
    """
    
    # Get project directory
    project_dir = env.get("PROJECT_DIR")
    build_dir = env.get("BUILD_DIR")
    
    # Paths
    bin_file = os.path.join(build_dir, "firmware.bin")
    uf2_file = os.path.join(project_dir, "PomeloCore.uf2")
    uf2conv_script = os.path.join(project_dir, "uf2conv.py")
    
    # Check if uf2conv.py exists
    if not os.path.exists(uf2conv_script):
        print("WARNING: uf2conv.py not found at:", uf2conv_script)
        print("UF2 file will not be generated")
        return
    
    # Check if bin file exists
    if not os.path.exists(bin_file):
        print("WARNING: firmware.bin not found at:", bin_file)
        return
    
    # Run uf2conv.py
    try:
        print("Converting firmware.bin to UF2 format...")
        result = subprocess.run(
            ["python3", uf2conv_script, bin_file, "-o", uf2_file],
            capture_output=True,
            text=True,
            check=True
        )
        
        if result.returncode == 0:
            print(f"✓ UF2 file created: {uf2_file}")
            
            # Print file size
            if os.path.exists(uf2_file):
                size = os.path.getsize(uf2_file)
                print(f"  Size: {size} bytes ({size/1024:.1f} KB)")
        else:
            print("ERROR: uf2conv.py failed")
            print(result.stderr)
            
    except subprocess.CalledProcessError as e:
        print(f"ERROR running uf2conv.py: {e}")
        print(e.stderr)
    except Exception as e:
        print(f"ERROR: {e}")

# Register the post-build action
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)

print("Post-build script registered: UF2 conversion enabled")
