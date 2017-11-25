1.txt: 
env: obstacle
n = 2
link length = 1,2
start state = 0,0,0,0
end state = 1,1,1,1
velocity limit = 10
torque limit = 100
time limit = 60

2.txt:
env: free space
n = 1
link length = 1
start state = 0,0
end state = 1,1
velocity limit = 10
time limit = 60
comment: To test the correctness of the simple pendulum environment

3.txt:
env:obstacle
n = 3
link length = 1,2,3
start state = 0,0,0,0,0,0
end state = 3,0,0,0,0,0
velocity limit = 10
torque limit = 60
time limit = 200
note: it is not a complete solution, however, the experiment shows that when n =3, the time needed for RRT space will increases rapidly.

4.txt
env:free space
n = 2
link length = 1,1
start state = 0,0,0,0
end state = 3,0,0,0
velocity limit = 10
torque limit = 100
time limit = 100

5.txt
env:obstacle
n = 2
link length = 1,1
start state = 0,0,0,0
end state = 3,0,0,0
velocity limit = 10
torque limit = 100
time limit = 100
note: it is not perfect in the end, however it is a approximate solution, as we can see, the planner is approaching the final state

6 free space
n=3
length input 0.3, 0.6, 0.9
start state 0, 0, 0, 0, 0, 0
end state 3, 0, 0, 0, 0, 0
velocity limit 10
torque limit 100
time limit 100

7 free space
n=3
length input 0.3, 0.6, 0.9
start state 0, 0, 0, 0, 0, 0
end state 3, 0, 0, 0, 0, 0
velocity limit 10
torque limit 100
time limit 200

8 with obstacle
n=3
length input 0.3, 0.6, 0.9
start state 0, 0, 0, 0, 0, 0
end state 3, 0, 0, 0, 0, 0
velocity limit 10
torque limit 100
time limit 200


