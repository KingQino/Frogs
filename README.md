# Frogs



## Intro



## Programming Architecture

## Debug

```sh
qlogin -pe smp 2 -l h_vmem=16G -l h_rt=2:0:0 -l rocky 
```

```sh
ml cmake intel valgrind/3.20.0-intel-oneapi-mpi-2021.12.1-oneapi-2024.1.0
```

```sh
cmake -DCMAKE_BUILD_TYPE=Release ..
make valgrind_run
```



phenomenon: 

The longer it runs, the memory comsued more.  

> run 60 seconds:
>
> 1. 127,380,710 bytes allocated
> 2. 124,163,570 bytes allocated (20 seconds)
> 3. 223,184,310 bytes allocated (60 seconds) -> 167,643,882 bytes -> 167,643,882 bytes allocated -> 167,623,449 bytes allocated (turn off logging)
> 4. 

E-n23-k3 is a special case, 只有一个顾客的需求量特别大(4100)几乎达到汽车容量的上限（4500）,然而其他顾客的需求量都比较小。



## Usage

1. First step - compile

   ```sh
   git clone -b main git@github.com:KingQino/Frogs.git
   ```

   Apocrita:

   ```sh
   ml load cmake/3.23.1 gcc/12.1.0 openmpi/4.1.4-gcc
   ```

   Sulis:

   ```sh
   ml load CMake/3.18.4 GCC/13.2.0
   ```

   ```shell
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make
   ```

2. Second step - run

   ```shell
   ./Run 1 E-n22-k4.evrp 1 0
   
   # Explanation
   # ./Run <algorithm: 0 for CBMA, 1 for LAHC> <problem_instance_filename> <stop_criteria: 0 for max-evals, 1 for max-time> <multithreading: 0 for yes>
   ```

3. Hpc - run

   `./build/parameters.txt`

   ```sh
   E-n22-k4.evrp
   E-n23-k3.evrp
   E-n30-k3.evrp
   E-n33-k4.evrp
   E-n51-k5.evrp
   E-n76-k7.evrp
   E-n101-k8.evrp
   X-n143-k7.evrp
   X-n214-k11.evrp
   X-n351-k40.evrp
   X-n459-k26.evrp
   X-n573-k30.evrp
   X-n685-k75.evrp
   X-n749-k98.evrp
   X-n819-k171.evrp
   X-n916-k207.evrp
   X-n1001-k43.evrp
   ```

   `setup.sh`

   - **Apocrita**:

     ```sh
     #!/bin/bash
     
     # Target directories
     directories=$(ls -d */ | grep -Ev '^(A|B)/')
     
     # Check if any directories were found
     if [ -z "$directories" ]; then
         echo "No directories found."
         exit 1
     fi
     
     # Loop through each directory to set up build structure and create script.slurm
     for dir in $directories; do
         # Remove trailing slash from directory name
         dir=${dir%/}
     
         # Check if the directory exists
         if [ -d "$dir" ]; then
             # Define paths for build and log folders
             build_dir="$dir/build"
             log_dir="$build_dir/log"
     
             # Create the build and log directories if they don't exist
             mkdir -p "$log_dir"
             echo "Created '$log_dir'."
     
             # Copy parameters.txt into the build folder
             if [ -f "parameters.txt" ]; then
                 cp parameters.txt "$build_dir"
                 echo "Copied 'parameters.txt' to '$build_dir'."
             else
                 echo "'parameters.txt' not found in the current directory."
             fi
     
             # Create script.slurm with dynamic content
             cat > "$build_dir/script.sh" <<EOL
     #!/bin/bash
     #$ -pe smp 10
     #$ -l rocky
     #$ -l h_vmem=1G
     #$ -l h_rt=48:0:0
     #$ -cwd
     #$ -j y
     #$ -N $dir
     #$ -o $(pwd)/$log_dir
     #$ -t 1-17
     #$ -tc 17
     
     module load gcc/12.2.0
     
     set -e
     
     mapfile -t cases < parameters.txt
     CASE="\${cases[\$((SGE_TASK_ID - 1))]}"
     
     # Check if CASE is valid
     if [ -z "\$CASE" ]; then
         echo "Error: No parameter found for SGE_TASK_ID=\$SGE_TASK_ID" >&2
         exit 1
     fi
     
     echo "Running task \$SGE_TASK_ID with CASE=\$CASE"
     
     ./Run -alg lahc -ins "\$CASE" -log 1 -stp 1 -mth 1
     EOL
     
             echo "Generated 'script.sh' in '$build_dir'."
     
             # Navigate to build_dir, run cmake and make commands
             (
                 cd "$build_dir" || exit
                 cmake -DCMAKE_BUILD_TYPE=Release .. 
                 make
                 qsub script.sh
             )
         else
             echo "Directory '$dir' does not exist."
         fi
     done
     ```

   - **Sulis**

     ```sh
     #!/bin/bash
     
     # Target directories
     directories=$(ls -d */ | grep -Ev '^(A|B)/')
     
     # Check if any directories were found
     if [ -z "$directories" ]; then
         echo "No directories found."
         exit 1
     fi
     
     # Loop through each directory to set up build structure and create script.slurm
     for dir in $directories; do
         # Remove trailing slash from directory name
         dir=${dir%/}
     
         # Check if the directory exists
         if [ -d "$dir" ]; then
             # Define paths for build and log folders
             build_dir="$dir/build"
             log_dir="$build_dir/log"
     
             # Create the build and log directories if they don't exist
             mkdir -p "$log_dir"
             echo "Created '$log_dir'."
     
             # Copy parameters.txt into the build folder
             if [ -f "parameters.txt" ]; then
                 cp parameters.txt "$build_dir"
                 echo "Copied 'parameters.txt' to '$build_dir'."
             else
                 echo "'parameters.txt' not found in the current directory."
             fi
     
             # Create script.slurm with dynamic content
             cat > "$build_dir/script.slurm" <<EOL
     #!/bin/bash
     
     # Slurm job options (job-name, compute nodes, job time)
     #SBATCH --job-name=Lahc-$dir                             # Job name set to the parent directory name
     #SBATCH --output=$(pwd)/$log_dir/slurm-%A_%a.out       # Output log file path in the log folder
     #SBATCH --time=48:0:0                                  # Request 48 hours of compute time
     #SBATCH --nodes=1                                      # Request 1 node
     #SBATCH --tasks-per-node=1                             # One task per node
     #SBATCH --cpus-per-task=10                             # Each task uses 10 CPUs (threads)
     #SBATCH --mem-per-cpu=1G                               # Memory per CPU
     #SBATCH --account=su008-exx866
     #SBATCH --array=0-16
     
     # Load necessary modules
     module load GCC/13.2.0
     
     # Load cases from parameters.txt
     mapfile -t cases < "parameters.txt"        # Load parameters.txt from the build directory
     CASE="\${cases[\$SLURM_ARRAY_TASK_ID]}"
     
     # Run the specified command with case argument
     srun ./Run 1 "\$CASE" 1 0
     EOL
     
             echo "Generated 'script.slurm' in '$build_dir'."
     
             # Navigate to build_dir, run cmake and make commands
             (
                 cd "$build_dir" || exit
                 cmake -DCMAKE_BUILD_TYPE=Release ..
                 make
                 sbatch script.slurm
             )
         else
             echo "Directory '$dir' does not exist."
         fi
     done
     ```

4. Statistics

   `objective.sh`

   ```sh
   #!/bin/bash
   
   output_file="a.txt"
   > "$output_file" # Clear or create the output file
   
   # List of directories and their specific stats files in the desired order
   declare -A stats_files=(
       [E-n22-k4]="stats.E-n22-k4.evrp"
       [E-n23-k3]="stats.E-n23-k3.evrp"
       [E-n30-k3]="stats.E-n30-k3.evrp"
       [E-n33-k4]="stats.E-n33-k4.evrp"
       [E-n51-k5]="stats.E-n51-k5.evrp"
       [E-n76-k7]="stats.E-n76-k7.evrp"
       [E-n101-k8]="stats.E-n101-k8.evrp"
       [X-n143-k7]="stats.X-n143-k7.evrp"
       [X-n214-k11]="stats.X-n214-k11.evrp"
       [X-n351-k40]="stats.X-n351-k40.evrp"
       [X-n459-k26]="stats.X-n459-k26.evrp"
       [X-n573-k30]="stats.X-n573-k30.evrp"
       [X-n685-k75]="stats.X-n685-k75.evrp"
       [X-n749-k98]="stats.X-n749-k98.evrp"
       [X-n819-k171]="stats.X-n819-k171.evrp"
       [X-n916-k207]="stats.X-n916-k207.evrp"
       [X-n1001-k43]="stats.X-n1001-k43.evrp"
   )
   
   # Process files in the given sequence
   for dir in E-n22-k4 E-n23-k3 E-n30-k3 E-n33-k4 E-n51-k5 E-n76-k7 E-n101-k8 \
              X-n143-k7 X-n214-k11 X-n351-k40 X-n459-k26 X-n573-k30 X-n685-k75 \
              X-n749-k98 X-n819-k171 X-n916-k207 X-n1001-k43; do
       file_path="$dir/${stats_files[$dir]}"
       if [ -f "$file_path" ]; then
           tail -n 3 "$file_path" >> "$output_file"
       else
           echo "File not found: $file_path" >&2
       fi
   done
   
   # Process the output file to ensure results follow the sequence
   awk '
   /Mean/ {
       mean_value = $2;
       std_dev_value = $NF;
   }
   /Min:/ {
       min_value = $2;
       print min_value;
       print mean_value;
       print std_dev_value;
   }' "$output_file"
   
   rm -f "$output_file"
   ```

   

   