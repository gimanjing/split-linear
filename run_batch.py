import os
import subprocess
import csv
from pathlib import Path

# Configuration
PROGRAM_DIR = "./Program"
EXECUTABLE = os.path.join(PROGRAM_DIR, "split")
SOLVERS = ["PTVRP", "PTVRP_LINEAR", "PTVRP_LAYERED"]
FOLDERS = [
    "../Instances/Instances 1",
    "../Instances/Instances 2",
    "../Instances/Instances 3"
]
OUTPUT_CSV = "ptvrp_batch_results.csv"
LOGS_DIR = "ptvrp_logs"

# Ensure log directory exists
Path(LOGS_DIR).mkdir(parents=True, exist_ok=True)

results = []

# Ensure the binary is built
print("Building the solver...")
subprocess.run(["make"], cwd=PROGRAM_DIR, check=True)

for folder in FOLDERS:
    folder_path = Path(folder)
    if not folder_path.exists():
        print(f"Directory not found: {folder}")
        continue
    
    # Find all instance files (e.g., .gt or whatever extension they use)
    instance_files = sorted(list(folder_path.glob("**/*")))
    instance_files = [f for f in instance_files if f.is_file() and not f.name.startswith('.')]
    
    print(f"Processing {folder}: found {len(instance_files)} files.")
    
    for inst_file in instance_files:
        relative_inst_path = str(inst_file)
        
        for solver in SOLVERS:
            cmd = [EXECUTABLE, relative_inst_path, "-solver", solver]
            
            try:
                # Run the solver with a timeout (e.g., 60 seconds per run)
                res = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=60
                )
                stdout = res.stdout
                stderr = res.stderr
                return_code = res.returncode
            except subprocess.TimeoutExpired:
                stdout = ""
                stderr = "TIMEOUT"
                return_code = -1
            except Exception as e:
                stdout = ""
                stderr = str(e)
                return_code = -999

            # Save individual log for deep inspection later
            safe_name = f"{folder_path.name}_{inst_file.stem}_{solver}.txt"
            log_file_path = Path(LOGS_DIR) / safe_name
            with open(log_file_path, "w") as f:
                f.write(f"Command: {' '.join(cmd)}\n")
                f.write(f"Return Code: {return_code}\n")
                f.write("--- STDOUT ---\n")
                f.write(stdout)
                f.write("\n--- STDERR ---\n")
                f.write(stderr)

            # Record summary data for CSV
            results.append({
                "folder": folder_path.name,
                "instance": inst_file.name,
                "solver": solver,
                "return_code": return_code,
                "log_file": str(log_file_path),
                "raw_output": stdout.replace("\n", " ")  # Flattened for CSV
            })

# Write aggregated summary to CSV
keys = ["folder", "instance", "solver", "return_code", "log_file", "raw_output"]
with open(OUTPUT_CSV, "w", newline="", encoding="utf-8") as csv_file:
    writer = csv.DictWriter(csv_file, fieldnames=keys)
    writer.writeheader()
    writer.writerows(results)

print(f"Batch execution complete. Summary saved to {OUTPUT_CSV}, individual logs in {LOGS_DIR}/.")
