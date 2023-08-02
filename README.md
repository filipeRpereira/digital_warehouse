# digital_warehouse
A Path Optimization Algorithm for Industrial Robots Using Deep Reinforcement Learning

### About this repository
Lorem ipsum dolor sit amet, consectetur adipiscing elit. Donec a finibus orci, pulvinar luctus arcu. Praesent pharetra 
condimentum est sit amet molestie. Suspendisse efficitur cursus interdum. Mauris ac justo id libero cursus auctor 
vitae vel lacus. Nam egestas ac mi eu viverra. In a neque nulla. Praesent sed lectus sollicitudin, vehicula est sit 
amet, elementum neque. Integer et lorem accumsan, faucibus nulla quis, imperdiet libero. Curabitur efficitur faucibus 
nunc. Orci varius natoque penatibus et magnis dis parturient montes, nascetur ridiculus mus. Nam interdum dolor massa, 
nec accumsan augue aliquet vitae. Maecenas ac ipsum finibus, commodo dolor vel, ullamcorper leo. Ut id blandit urna. 
Suspendisse sapien sem, mattis semper mi in, ullamcorper commodo dui. Maecenas at pellentesque magna, sodales pretium 
odio. Praesent lacinia aliquam augue at ornare.

Aliquam dignissim lacinia volutpat. Morbi tincidunt quis enim et semper. Donec eget finibus orci, vestibulum posuere 
nulla. Vivamus placerat tincidunt purus, eget accumsan leo pharetra non. Mauris eget vestibulum est. Proin tempor 
ipsum at sapien dictum, at finibus arcu ultricies. Praesent dignissim non elit vel tristique. Cras id vestibulum risus. 
Vivamus velit magna, dapibus in accumsan in, blandit eu eros. Aliquam maximus venenatis sem, ut porttitor sapien 
hendrerit eget. Nulla ullamcorper vestibulum nisi a eleifend. Maecenas fringilla congue nulla feugiat lobortis.

### Installation
Vivamus aliquam eros in iaculis hendrerit. Pellentesque maximus urna eget nibh congue, sit amet commodo massa ultrices. 
Etiam lacinia id diam non suscipit. Sed mi enim, convallis posuere ultricies sit amet, convallis nec orci. Suspendisse 
rhoncus orci in pharetra rutrum. Maecenas ac eleifend quam, consectetur efficitur quam. Duis orci justo, aliquet a 
iaculis sit amet, aliquet id mi. Pellentesque vehicula sapien mauris. Aliquam lacus tellus, porttitor non lorem sed, 
semper dignissim neque.

#### Miniconda
[website](https://docs.conda.io/en/latest/miniconda_hashes.html)

#### Isaac Gym Preview 4
Download the Isaac Gym Preview 4 release from the [website](https://developer.nvidia.com/isaac-gym), then
follow the installation instructions in the documentation. We highly recommend using a conda environment 
to simplify set up.

#### Isaac Gym Benchmark Environments
[website](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)\
Once Isaac Gym is installed and samples work within your current python environment, install this repo:

```bash
pip install -e .
```


#### Isaac Sim
[website](https://developer.nvidia.com/isaac-sim/download)

#### tensorboard
`$ pip install tensorboard`

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


### Loading trained models // Checkpoints
Fusce egestas mauris et justo ornare hendrerit. Etiam pellentesque nec enim vitae semper. Praesent consequat magna quis 
pulvinar venenatis. Sed gravida nisi nulla, nec mattis tellus consequat vitae. In eu varius orci, sit amet hendrerit 
neque. Phasellus lorem leo, malesuada eget pellentesque ac, elementum ut dolor. Ut viverra quis elit a lobortis. 
Vestibulum consectetur, tellus sit amet molestie cursus, magna velit laoreet erat, sed rhoncus nisl nulla nec dolor. 
Maecenas efficitur lectus at neque fermentum, aliquam iaculis tellus euismod. Aliquam bibendum eget magna at lobortis. 
Nullam sed turpis ut leo gravida fermentum. Sed blandit, nulla a pulvinar hendrerit, eros elit suscipit justo, eu 
venenatis massa orci sed turpis. Curabitur euismod consectetur metus nec ornare.

### Configuration and command line arguments

* `task=TASK` - selects which task to use. Any of `AllegroHand`, `Ant`, `Anymal`, `AnymalTerrain`, `BallBalance`, 
* `Cartpole`, `FrankaCabinet`, `Humanoid`, `Ingenuity` `Quadcopter`, `ShadowHand`, `ShadowHandOpenAI_FF`, 
* `ShadowHandOpenAI_LSTM`, and `Trifinger` (these correspond to the config for each environment in the folder 
* `isaacgymenvs/config/task`)

## Tasks

