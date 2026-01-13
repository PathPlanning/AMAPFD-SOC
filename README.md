# Solving Anonymous Multi-Agent Path Finding Problem with Disappearing Agents (AMAPFD) problem with Sum-Of-Cost (SOC) cost function.


Solve AMAPFD problem using Minimum Cost Maximum Flow (MCMF) with General Bulk-Search (GBS) algorithm over A* algorithm for finding the solution with minimal Sum-Of-Cost.

The code is written with 'light' level of comments. However, it maintains the flow and notations of the paper.

## Folders
1. `data` contains the maps.
2. `results` where to store the results.


## Build and Run
The package was tested and run on `Ubuntu 22.04.2`.\
Build the package by:
```
mkdir build
cd build
cmake ..
make
```

All tests used in the paper (all MAPF maps, all scenarios, the number of agents changes by the sequence 1,2,3,4,5,6,7,8,9,10,50,100,150,...,min(1000, max_number_of_agents)), are hard-coded in the main function, and ready to run.

Run the program by the command:
```
./amapfd_soc_mcmf_gbs_astar
```

## Results
The data of the SOC-optimal solution of each instance are logged in a csv-style file (`results/results.csv`).