import os
import sys

def verify_dataset():
    # The dataset was cloned into IO-VNBD in the root directory
    dataset_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'IO-VNBD')
    
    if not os.path.exists(dataset_dir):
        print(f"Dataset directory not found at {dataset_dir}")
        return False
        
    print(f"Dataset directory found: {dataset_dir}")
    
    expected_subdirs = ['Synchronised V abd S datasets', 'Unsynchronised V and S Dataset']
    for subdir in expected_subdirs:
        subdir_path = os.path.join(dataset_dir, subdir)
        if os.path.exists(subdir_path):
            print(f"[OK] Found expected directory: {subdir}")
            
            # Count files
            files = os.listdir(subdir_path)
            print(f"     - Contains {len(files)} files/folders")
        else:
            print(f"[ERROR] Missing expected directory: {subdir}")
            return False
            
    print("\nDataset Verification Passed. Structure is intact.")
    return True

if __name__ == '__main__':
    success = verify_dataset()
    sys.exit(0 if success else 1)
