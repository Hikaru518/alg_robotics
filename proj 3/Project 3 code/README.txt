Excel Files:
C_sampler_results_TimeLimit50s: results in Cubicles environment with 50s time limit.
T_sampler_results_TimeLimit50s: results in Twisty-cool environment with 50s time limit.

About the source code:
For the problem 1&2, the main function is located in “./env1(env2)/MyRigidBodyPlaning.cpp”, and RTP planner is in “RTP.cpp”

For benchmark problem, the main function is located in “./benchmark/src/SE3RigidBodyPlanningBenchmark.cpp”, RTP planner is in “RTP.cpp”

How to run the program:
For Environment 1. Go to “env1” folder, and type:
$ make clean
$ make
$ ./MyRigidBodyPlanning
To visualize the path, type:
$ ./plotSolution.py

For Environment 2. Go to “env2” folder, and type:
$ make clean
$ make
$ ./MyRigidBodyPlanning
To visualize the path, type:
$ ./plotSolution.py

For Benchmark Problem. Go to “benchmark” folder, and type:
$ make clean
$ make
To benchmark in Cubicles scenario, type: 
$ ./SE3RigidBodyPlanningBenchmark 0 0
To benchmark in Cubicles scenario, type: 
$ ./SE3RigidBodyPlanningBenchmark 1 0
To generate pdf report, type:
$ sh generatePDF.sh
and the result pdf files can be found at “result_pdf” folder.

