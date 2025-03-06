# Frogs

- runtime environment:

  > Calculations were performed using the __Sulis__ Tier 2 HPC platform hosted by the Scientific Computing Research Technology Platform at the University of Warwick. Sulis is funded by EPSRC Grant EP/T022108/1 and the HPC Midlands+ consortium.

  - CPU: AMD EPYC 7742 (Rome) 2.25 GHz
  - Memory request: 350MB (instance X-n916 - 314424K - LAHC)

- a

## Debug & Obvservation

1. open an interactive job, login in a specific node

   ```sh
   qlogin -pe smp 2 -l h_vmem=16G -l h_rt=1:0:0 -l rocky 
   ```

   ```sh
   ml cmake intel valgrind/3.20.0-intel-oneapi-mpi-2021.12.1-oneapi-2024.1.0
   ```

2. make some changes at  `CMakeLists.txt` and `preprocessor.cpp`

   ```cmake
   # Custom target to run Valgrind with algorithm arguments
   add_custom_target(valgrind_run
           COMMAND valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --error-exitcode=1 --log-file=valgrind_run.log
           ./Run -alg lahc -ins X-n916-k207.evrp -log 1 -stp 1 -mth 0
           DEPENDS Run
           COMMENT "Running Valgrind on executable Run with algorithm parameters..."
   )
   ```

   ```c++
   max_exec_time_ = 20; // 20 seconds
   ```

3. run __valgrind__

   ```sh
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make valgrind_run
   ```

4. check memory usage

   ```sh
   # Apocrita
   jobstats
   
   # Sulis
   sacct -j 1087552 --format=JobID,JobName,Partition,AllocCPUs,MaxRSS,ReqMem,Elapsed,State
   ```

   

Bug-fix log:

> memory leak and explosion have been solved.
>
> Case 1: The longer the algorithm runs, the memory it comsumes more.  => memory leak and explosion
>
> - run 20 seconds
>
>   124,163,570 bytes allocated
>
> - run 60 seconds
>
>   223,184,310 bytes allocated -> 167,643,882 bytes (vector shrink_to_fit)  -> 167,643,882 bytes allocated () -> 167,623,449 bytes allocated (turn off logging)
>
> - `E-n23-k3` is a special instance where a single customer has an exceptionally high demand (4100), nearly reaching the vehicle’s capacity limit (4500), while the demands of all other customers remain relatively low.



## Usage

1. First step - compile

   ```sh
   git clone -b main git@github.com:KingQino/Frogs.git main
   ```

   ```sh
   # Apocrita
   ml load cmake gcc openmpi
   
   # Sulis
   ml load CMake/3.18.4 GCC/13.2.0
   ```

   ```shell
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release ..
   make
   ```
   
2. Second step - run

   ```shell
   ./Run -alg lahc -ins E-n22-k4.evrp -log 1 -stp 1 -mth 1
   
   '
   -------------------------------------------------- Parameters Instruction  --------------------------------------------------
   Usage: ./Run [options]
   Options:
     -alg [enum]                  : Algorithm name (e.g., Cbma, Lahc)
     -ins [filename]              : Problem instance filename
     -log [0|1]                   : Enable logging (default: 0)
     -stp [0|1|2]                 : Stopping criteria, 0: max-evals, 1: max-time, 2: obj-converge (default: 0)
     -mth [0|1]                   : Enable multi-threading (default: 1)
     -seed [int]                  : Random seed (default: 0)
     -nb_granular [int]           : Granular search parameter (default: 20)
     -is_hard_constraint [0|1]    : Whether to use hard constraint (default: 1)
     -is_duration_constraint [0|1]: Whether to consider duration constraint (default: 0)
     -history_length [int]        : LAHC history length (default: 5000)
   -----------------------------------------------------------------------------------------------------------------------------
   '
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
     #SBATCH --partition=compute                            # Use compute partition - all CPU nodes are the same
     #SBATCH --job-name=L-$dir                              # Job name set to the parent directory name
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

   


## Programming Architecture

- Data Handling Layer

  - `Case`: Stores **raw CEVRP instance data**
  - `Preprocessor`: Computes **preprocessed data**, used in the optimisation process
  - `Individual`: Stores **solutions** (routes for vehicles).

- Optimisation Layer

  - `Split`: Transfer chromsome into upper-level subsolution

  - `Leader`: Optimizes **route structures**, ensuring **vehicle capacity feasibility** but **ignores charging decisions**, i.e., `Localsearch`
  - `Follower`: Makes **charging decisions**, ensuring **route feasibility for electric vehicles** while maintaining the given path.

- Heuristic Algorithm Layer

  - `HeuristicInterface`: Abstract class defining the structure for **heuristic algorithms**.
  - `Ma`: evolutionary optimisation
  - `Lahc`: single-point based algorithm

- Statistical Analysis Layer

  - `StatsInterface`: Provides methods for **tracking algorithm performance**, recording iteration details, and analyzing convergence.

- Command-Line Interface

  - `CommandLine`: Parses **runtime parameters** (e.g., `-nbGranular 20`, `-maxIter 1000`, `-algorithm Ma`).
  - `Parameters`: parse arguments from command line to the parameters. 

---

- How does `Individual`  interact with <u>Optimisation Layer</u>?

  `Individual` contains members:

  - chromosome (`chromT`)

  - upper solution (`chromR`)

  - CostSol (penalisedCost, nbRoutes, distance, capacityExcess, durationExcess)

  - Lower_cost

    > `Lahc` only use the above 5 members, but `Ma` will use all these members

  - successors

  - predecessors

  - indivsPerProximity

  - isFeasible

  - biasedFitness

  > Initiased `Individual` => contains chromT
  >
  > - `Split` => chromR and CostSol
  > - `LocalSearch` => chromT, chromR, CostSol
  >   - Linked list
  >   - Import and export
  >   - export to `Follower` data structure
  > - `Follower` => lower_cost
  >   - int** lower_routes;
  >   - import and export

- how to advoid some moves that have been already tested without improvement?

  - In `Lahc`, temporarily ignore it.
  - In `Ma`, we can consider to use it!

- how to make the lower-level optimisation?

  - In `Follower`, load individual to its data structure, then optimising. in the end, the updated cost is exported to the `Inidivual`
    - int ** lower_routes;
    - int * lower_num_nodes_per_routes;
    - double lower_cost;
  - To output the final result, we just need to make the lower-level optimisation to the `Individual` upper-level solution again, and then we can get the exactly same solution. 

- In `Lahc`, it's very resource-comsuing for making charging decision, so maybe we can remove some unnecessary lower-level optimisation. 

  - No lower-level optimisation in the first $n * L_h$ iterations, based on the assumption that the solution  has not converge enough, and the upper-level soluton is positively related to the lower-level solution. 
  - if current solution upper cost ($\mathbf{x}^u$) is less than $\eta$ multiplied by the best upper cost so far ($\eta \cdot \mathbf{x}^u_g$) , then make the lower-level optimisation. 

- In `Lahc`, how to make moves and update the current solution?

  - we don't need to copy current solution and make the lower-level optimisation, what we need is just make updates on the current solution itself.

---

- `Leader` implementation details

  1. randomly select node U
  2. randomly select node V from the correlated vertices of U
  3. randomly select a move from the 9 moves
     - if `routU` = `routeV`, then M1-7. 
       - No need to check vehicle capacity constraints
       - Or to be consisent with the current Lahc, only M1, M4, M7
     - if `routeU` != `routeV`, then M1-6, M8, M9
       - Or to be consistent with the current Lahc, only M1, M4, M8, M9
       - If not satisfy the vehicle capacity constraint, then don't move
     - If consective 10 times, no improvement, then finished 
  4. apply the move, change the acceptance criteria

  > - For each move, the cost change should be tracked
  > - For each move, it should have two outputs:
  >   - only the objective cost (upper-cost), just taken from the `Leader`
  >   - Indvidual (especially `chromR`) , so we can apply the lower-level optimisation
  > - When there are some empty routes, set node V as the depot of the empty route, then M1, 2, 3 and 9 can be applied. In this case, a route will be created.
  > - If node V is depot,  then we can apply M1, 2, 3, 8, 9. 

  
