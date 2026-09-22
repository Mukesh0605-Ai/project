# Manual Dataset Download Required

## 1. Exact Official Page
The official dataset repository is located at:
**[https://github.com/onyekpeu/IO-VNBD](https://github.com/onyekpeu/IO-VNBD)**

## 2. Exact Dataset / Archive Required
The complete `IO-VNBD` (Inertial Odometry Vehicle Navigation Benchmark Dataset).

**Note:** The `git clone` command only pulled the folder structure and a single sample file (`V-Vfa02.csv`). The remaining `.csv` files for the Smartphone dataset and other Vehicle dataset drives are missing from the GitHub repository itself (likely due to missing Git LFS objects or the author not uploading them directly to the repo). 

You must manually obtain the full dataset from the author (e.g., via a secondary storage link provided by the author upon request) since it is not hosted directly on the GitHub releases or repository tree.

## 3. Exact Files Expected
You need the full suite of `.csv` files containing the 6-DOF IMU data. Specifically, you need the files that populate the `S Dataset` (Smartphone data) and the full `V Dataset` directories. 

## 4. Where to Place Them
Once downloaded, extract the files and place them directly into:
`data/raw/io-vnbd/`

Ensure the directory structure looks like this:
`data/raw/io-vnbd/Synchronised V abd S datasets/`
`data/raw/io-vnbd/Unsynchronised V and S Dataset/`

## 5. Verification Command
Once the files are placed, run the following Python verification script to confirm the structure and file count:
```bash
python scripts/verify_dataset.py
```
*(Ensure you update `verify_dataset.py` to point to `data/raw/io-vnbd` if it doesn't already).*
