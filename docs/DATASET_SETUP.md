# Dataset Setup

**STATUS: DATASET_READY**

## 1. Dataset required
IO-VNBD (Inertial Odometry Vehicle Navigation Benchmark Dataset)

## 2. Official source
https://github.com/onyekpeu/IO-VNBD

## 3. Exact download procedure
The dataset has been successfully cloned into the workspace via git:
`git clone https://github.com/onyekpeu/IO-VNBD.git`

## 4. Expected files
The repository contains:
- `README.md`
- `README_1.pdf`
- `Synchronised V abd S datasets/`
- `Unsynchronised V and S Dataset/`

## 5. Expected directory structure
The dataset sits at `IO-VNBD/` in the root of the project.

## 6. What needs to be placed inside this project
The dataset has already been placed in the project root. Future processing scripts should reference paths within the cloned `IO-VNBD` directory.

## 7. Verification procedure
Run `python scripts/verify_dataset.py` to ensure the structure exists and is intact.
