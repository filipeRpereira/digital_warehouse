# digital_warehouse
## A Path Optimization Algorithm for Industrial Robots Using Deep Reinforcement Learning
![Warehouse_image.png](Images%2FWarehouse_image.png)
### About this repository
Energy efficiency has become an increasingly relevant concern in the industry, with the aim of reducing environmental 
impact and operating costs. In this work, energy consumption optimization solutions are presented in robotic arms 
during the execution of their tasks. The objective is to reduce the energy consumption of industrial robots, taking 
into account the importance of energy efficiency in the current context, where sustainability and the reduction of 
carbon emissions are priority topics.

To achieve this goal, reinforcement learning algorithms will be used, a Machine Learning approach that allows robots 
to learn and adapt to the best strategies to optimize their tasks, taking into account energy consumption. 
Through this learning process, it is expected to find the most efficient configurations and behaviors to reduce energy 
consumption, maintaining the quality and effectiveness of the tasks performed by the robotic arms.

The implementation of the proposed solution will be carried out using the Isaac Sim and Isaac Gym simulation platforms, 
together with the ROS framework (Robot Operating System). These tools will provide a suitable virtual environment for 
the development and testing of energy optimization strategies, allowing the performance evaluation and validation of
the obtained results.

Through the solution found in this work, it was possible to optimize pick-and-place tasks performed by an industrial 
robotic arm. It was possible to demonstrate that it is possible to obtain significant gains with 
reinforcement learning solutions without compromising the correct execution and without harming the productivity of 
the robotic arm.

### Installation
Here you can find the packages and software that will be necessary to run this project.
#### Miniconda
[website](https://docs.conda.io/en/latest/miniconda_hashes.html)\
It is important to install miniconda to install Isac Gym.

#### Isaac Gym Preview 4
[website](https://developer.nvidia.com/isaac-gym)\
Download the Isaac Gym Preview 4, then follow the installation instructions in the documentation. 

#### Isaac Gym Benchmark Environments
[website](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)\
Once Isaac Gym is installed and samples work within your current python environment, install this repo:


#### Isaac Sim
[website](https://developer.nvidia.com/isaac-sim/download)\
In this link, you will find the download file.

#### Tensorboard
`$ pip install tensorboard`\
To install the Tensorboard in Ubuntu, execute the command line above.

#### ROS
[website](http://wiki.ros.org/Installation/Ubuntu)

### Running the tests

#### Isaac Gym
First will be necessary running the commands line:\
`$ cd IsaacGymEnvs-main/isaacgymenvs`\
`$ conda activate rlgpu`\
`$ export LD_LIBRARY_PATH=/home/filipe/miniconda3/envs/rlgpu/lib:$LD_LIBRARY_PATH`\
`$ rocore`\
This will initialize the conda environment `rlgpu` and the `roscore`. 

For testing the final results (e.g. task 2), use the follow command line. If you want to select another 
task or result, change the task number or test number in the command line.\
ALso is possible change the number of environments.\
`$ python train.py task=FrankaCubeStack num_envs=512 
checkpoint=runs/FrankaCubeStack/nn/task_1/last_FrankaCubeStack_ep_task_1_fase_2.pth test=True`

To activate tensorboard, run the follow command line:\
`$ python -m tensorboard.main --logdir runs/FrankaCubeStack/summaries/`

#### Isaac Sim
First open the Isaac Sim and run the project:\
`ros_workspace/src/digital_warehouse/isaac_sim/franka_robot.usd`

To simulate the in the Isaac Sim, first run the following command lines:\
`$ cd ros_workspace/`\
`$ source devel/setup.bash`\
`$ cd src/digital_warehouse/scripts/`

To evaluate the results of Isaac Sim, you can you the script:\
`$ python script/ros_tools_isaac_sim.py`\
To is possible to analyse the data, is necessary that the Isaac Gym scrypt `franka_cube_stack.py` is running. 
The Isaac Gym will publish the ROS data to the topic `joint_states` and the Isaac Sim will control the robot with 
that information, by subscribing that ROS topic.\
You can do that by running the following command line:\
`$ python train.py task=FrankaCubeStack num_envs=512 
checkpoint=runs/FrankaCubeStack/nn/task_1/last_FrankaCubeStack_ep_task_1_fase_2.pth test=True`

Key arguments to the `ros_tools_isaac_sim.py` script are:
* `--save_data` - Selects if you want to save ROS data to Json file. Type: `bool`
* `--job_name` - The json filename of results.
* `--job_name_2` - The json filename for the final result when you are analysing the data for comparing with 
initial results.
* `--read_data` - Read data from json file. Type: `bool`
* `--get_angular_acc` - Get the sum of angular acceleration of each joint. Type: `bool`
* `--plot_acc` - Plot the angular acceleration. Type: `bool`
* `--plot_joint_num` - Name of the joint to plot the angular acceleration. Type `string`.
* `--plot_effort` - Plot the effort. Type: `bool`
* `--plot_position` - Plot the position of each individual joint. Type: `bool`
* `--num_samples` - Number of ROS samples to analyse. Default `50`. Type: `string` 

## Tasks Results
### Task 1

![task_1.gif](Images%2Ftask_1.gif)


### Task 2
![task_2.gif](Images%2Ftask_2.gif)

### Task 3
![task_3.gif](Images%2Ftask_3.gif)