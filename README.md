# Frogs

- runtime environment:

  > Calculations were performed using the **Sulis** Tier 2 HPC platform hosted by the Scientific Computing Research Technology Platform at the University of Warwick. Sulis is funded by EPSRC Grant EP/T022108/1 and the HPC Midlands+ consortium.

  - CPU: AMD EPYC 7742 (Rome) 2.25 GHz
  - Memory request: To be evaluated 

- **Table of Contents**

  1. [Architecture](#Architecture)
  2. [Usage](#Usage)
  3. [Debug](#Debug)



## Architecture

- Data Handling Layer

  - `Case`: Stores **raw CEVRP instance data**
  - `Preprocessor`: Computes **preprocessed data**, used in the optimisation process
  - `Individual`: Stores **solutions** (routes for vehicles) - for `MA`
  - `Solution`: Stores solutions (for `Lahc`)

- Optimisation Layer

  - `Split`: Transfer chromsome into upper-level subsolution

  - `Initializer`: Solution initialisation methods

  - `Leader`: Optimizes **route structures**, ensuring **vehicle capacity feasibility** but **ignores charging decisions**, i.e., `Localsearch`
  - `Follower`: Makes **charging decisions**, ensuring **route feasibility for electric vehicles** while maintaining the given path.

- Heuristic Algorithm Layer

  - `HeuristicInterface`: Abstract class defining the structure for **heuristic algorithms**.
  - `Ma`: population-based algorithm
  - `Lahc`: single-point algorithm

- Statistical Analysis Layer

  - `StatsInterface`: Provides methods for **tracking algorithm performance**, recording iteration details, and analyzing convergence.

- Command-Line Interface

  - `CommandLine`: Parses **runtime parameters** (e.g., `-nbGranular 20`, `-maxIter 1000`, `-algorithm Ma`).
  - `Parameters`: parse arguments from command line to the parameters.

---

- How does `Solution` interact with <u>Optimisation Layer</u>?

  `Solution` includes 6 members:

  - int** routes;
  - int num_routes;
  - int* num_nodes_per_route;
  - int* demand_sum_per_route;
  - double upper_cost;
  - double lower_cost;

  > Interact with
  >
  > - `Initializer` => using Solution constructor
  >
  > - `Leader` => using function `load_solution` and `export_solution`, and all upper-level optimisation are compeleted in the `leader`
  >   - routes
  >   - num_routes
  >   - num_nodes_per_route
  >   - demand_sum_per_route
  >   - upper_cost
  >
  > - `Follower` => using function `load_solution` and `export_solution`, and all upper-level optimisation are compeleted in the `follower`
  >   - lower_cost

- Is it possible for us to advoid some repetitive moves that have been already tested without improvement?

  - In `Lahc`, ignore it
  - In `Ma`, we can consider to use it

- how to make moves (<u>upper-level optimisation</u>) and update the current solution?

  - we don't need to load a `solution` into the `leader` every time before making a move => too redundant
  - Instead, in the entire optimisation process, we load it once at the start of the optimisation, the `leader` then directly makes moves and updates the solution in place

- how to make the <u>lower-level optimisation</u>?

  - In `Follower`, load `solution` to its data structure, then optimising. In the end, the updated cost is exported to the `solution`
    - int ** lower_routes;
    - int * lower_num_nodes_per_routes;
    - double lower_cost;
  - To output the final result, we just need to make the lower-level optimisation to the `Solution` upper-level solution again, and then we can get the exactly same solution. 

- In `Lahc`, it's very resource-comsuing for making charging decision, so maybe we can remove some unnecessary lower-level optimisation. 

  - No lower-level optimisation in the first $n * L_h$ iterations, based on the assumption that the solution  has not converge enough, and the upper-level soluton is positively related to the lower-level solution. 
  - if current solution upper cost ($\mathbf{x}^u$) is less than $\eta$ multiplied by the best upper cost so far ($\eta \cdot \mathbf{x}^u_g$) , then make the lower-level optimisation.  :white_check_mark: => experiments have proved it's useful

- How to design the `Individual` in `Ma`?

  - chromosome (`chromT`)

  - upper solution (`chromR`)

  - CostSol (penalisedCost, nbRoutes, distance, capacityExcess, durationExcess)

  - Lower_cost

    > Can be extened to use the penalised objective function in the future by allowing infeasible solutions

  - successors

  - predecessors

  - indivsPerProximity

  - isFeasible

  - biasedFitness

    > Measure the population diversity on based broken-pairs distance to help us explore the search space more thoroughly


## Usage

1. First step - compile

   ```sh
   git clone -b main git@github.com:KingQino/Frogs.git main
   ```

   ```sh
   # Apocrita
   ml cmake gcc openmpi
   
   # Sulis
   ml GCCcore/13.3.0 CMake/3.29.3 GCC/13.3.0 OpenMPI/5.0.3
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
     -his_len [int]               : LAHC history length (default: 5000)
     -rt_mul [int]                : Runtime multiplier (default: 1)
     -nb_granular [int]           : Granular search parameter (default: 20)
     -is_hard_constraint [0|1]    : Whether to use hard constraint (default: 1)
     -is_duration_constraint [0|1]: Whether to consider duration constraint (default: 0)
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
     #SBATCH --job-name=L-$dir                             # Job name set to the parent directory name
     #SBATCH --output=$(pwd)/$log_dir/slurm-%A_%a.out       # Output log file path in the log folder
     #SBATCH --time=48:0:0                                  # Request 48 hours of compute time
     #SBATCH --nodes=1                                      # Request 1 node
     #SBATCH --tasks-per-node=1                             # One task per node
     #SBATCH --cpus-per-task=10                             # Each task uses 10 CPUs (threads)
     #SBATCH --mem-per-cpu=1G                               # Memory per CPU
     #SBATCH --account=su008-exx866
     #SBATCH --array=0-16
     
     # Load necessary modules
     module purge
     module load GCCcore/13.3.0 GCC/13.3.0 OpenMPI/5.0.3
     
     
     # Load cases from parameters.txt
     mapfile -t cases < "parameters.txt"        # Load parameters.txt from the build directory
     CASE="\${cases[\$SLURM_ARRAY_TASK_ID]}"
     
     # Run the specified command with case argument
     srun ./Run -alg lahc -ins "\$CASE" -log 1 -stp 1 -mth 1 
     EOL
     
             echo "Generated 'script.slurm' in '$build_dir'."
     
             # Navigate to build_dir, run cmake and make commands
             (
                 cd "$build_dir" || exit
                 ml GCCcore/13.3.0 CMake/3.29.3 GCC/13.3.0 OpenMPI/5.0.3
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

   `./stats/[Algorithm]/objective.sh`

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


## Debug

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
           ./Run -alg lahc -ins E-n23-k3.evrp -log 1 -stp 1 -mth 0
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


RAM Optimisation:

> The max memory consumption is 863M on the instance `X-n916-k207`
>
> run 10 seconds, `E-n22-k4.evrp`
>
> - 322,662,962 bytes allocated
>
> - release vector in initialize_heuristic() => 322,661,497 bytes allocated
>
> - change StatsInterface::calculate_statistical_indicators => 322,661,497 bytes allocated  => :negative_squared_cross_mark:
>
> - change the `recursive_charging_placement` => stack-based implementation => 956,288,525 bytes allocated => a little modification => 948,573,049 bytes allocated  => using the State within the function 939,879,833 bytes allocated
>
>   Stack-based implementation consumes more memory than recursion version does. This is probably due to the built-in optimisation of C++, compiler, and registers.  
>
> - disable  the logging: 322,661,497 bytes allocated => 322,639,886 bytes allocated



Bug-fix log:

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
>
>
> Case 2: Init with Direct Encoding in instance `E-n30-k3` got wrong solutions (Stop 3 days)
>
> - 345.95 -> much lower than the BKS -> but for all other instances, perform normally
>
> - Try to locate the bug, in the process of following `Leader` -> num_routes is not match with the real route
>
>   ```
>   Route Capacity: 12
>   Node Capacity: 58
>   Number of Routes: 3
>   Upper Cost: 1750.83
>   Number of Nodes per route (upper): 10 15 2 0 0 0 0 0 0 0 0 0
>   Demand sum per route: 4475 4425 0 0 0 0 0 0 0 0 0 0
>   Upper Routes:
>   Route 0: 0 2 5 3 15 17 14 22 13 0 0 0 0 0 0...
>   Route 1: 0 16 6 23 9 27 26 12 21 8 4 18 1 24 0 0...
>   Route 2: 0 0 0 0 0...
>   Route 3: 0 0 0 0 0...
>   ...
>   Route 11: 0 0 0 0 0...
>   ```
>
> - In the `Lahc`, try to check the initialisation with direct encoding in the` initialize_heuristic()` function
>
>   ```
>   ChromT: 2 5 3 15 17 14 22 13 16 6 23 9 27 26 12 21 8 4 18 1 24 11 29 28 7 20 10 19 25 
>   ChromR: 
>     Route 0: 2 5 3 15 17 14 22 13 
>     Route 1: 16 6 23 9 27 26 12 21 8 4 18 1 24 
>     Route 2: 
>     Route 3: 11 29 28 7 20 10 19 25 
>     Route 4: 
>     ...
>     Route 11:
>   Upper Cost: 
>     Penalised Cost: 1750.83
>     Number of Routes: 3
>     Distance: 1750.83
>     Capacity Excess: 0
>     Duration Excess: 0
>   Is Upper Feasible: 1
>   Lower Cost: 0
>   ```
>
> - Locate the issue in the `Split->initIndividualWithHienClustering`, fix it.
>
>
> Case 3: Segmentation fault   (core dumped) - memory explosion
>
> phenomonen: no memory leak,  `E-n51-k5` - run 10 seconds - 2,391,700,642 bytes allocated ~= 2.22GB 
>
> - `E-n23-k3` - run 10 seconds - 372,021,174 bytes  
>
>  - run longer - run 20s: 372,021,174 bytes  => same
>
>  - add Individual Destructor - 10s: 372,021,174 bytes => same
>
> - ==> route_cap = 10 * num_vehicles
>
> - Need to set up a route_cap of sufficient size, i.e., assume enough vehicles to support optimise

