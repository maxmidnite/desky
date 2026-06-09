Import("env")
import os
import shutil

def after_build(source, target, env):
    print("\n--- Custom Post-Build Action ---")
    print("Copying firmware.bin to release folder...")
    
    bin_file = str(target[0])
    release_dir = os.path.join(env.subst("$PROJECT_DIR"), "release")
    
    if not os.path.exists(release_dir):
        os.makedirs(release_dir)
        
    dest_file = os.path.join(release_dir, "firmware.bin")
    shutil.copy(bin_file, dest_file)
    print(f"Success! Firmware copied to: {dest_file}\n")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)