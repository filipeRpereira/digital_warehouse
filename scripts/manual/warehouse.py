from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.tasks import BaseTask
import numpy as np

class FrankaPlaying(BaseTask):
    #NOTE: we only cover here a subset of the task functions that are available,
    # checkout the base class for all the available functions to override.
    # ex: calculate_metrics, is_done..etc.
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._goal_position = np.array([0.1, -0.3, 0.18])
        self._task_achieved = False
        return

    # Here we setup all the assets that we care about in this task.
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        
        self._table = scene.add(FixedCuboid(prim_path="/World/table",
                                            name="table",
                                            position=np.array([-0.075, 0.0, 0.05]),
                                            scale=np.array([1.1, 1.0, 0.1]),
                                            color=np.array([0.5, 0.1, 0.1])))
        
        self._franka_base = scene.add(FixedCuboid(prim_path="/World/_franka_base",
                                            name="_franka_base",
                                            position=np.array([-0.48, 0.0, 0.15]),
                                            scale=np.array([0.23, 0.2, 0.125]),
                                            color=np.array([0.2, 0.4, 0.8])))
        
        self._cubeA = scene.add(DynamicCuboid(prim_path="/World/cubeA",
                                            name="cubeA",
                                            position=np.array([0.05, 0.3, 0.13]),
                                            scale=np.array([0.05, 0.05, 0.05]),
                                            color=np.array([0, 0, 1.0])))

        
        self._cubeB = scene.add(DynamicCuboid(prim_path="/World/cubeB",
                                            name="cubeB",
                                            position=np.array([0.1, -0.3, 0.13]),
                                            scale=np.array([0.05, 0.05, 0.05]),
                                            color=np.array([0, 0.4, 0.1])))
        

        
        scene.add_default_ground_plane()
        self._franka = scene.add(Franka(prim_path="/World/Fancy_Franka",
                                        name="fancy_franka",
                                        position=np.array([-0.45, 0.0, 0.215])))
        return

    # Information exposed to solve the task is returned from the task through get_observations
    def get_observations(self):
        cube_position, _ = self._cubeA.get_world_pose()
        current_joint_positions = self._franka.get_joint_positions()
        observations = {
            self._franka.name: {
                "joint_positions": current_joint_positions,
            },
            self._cubeA.name: {
                "position": cube_position,
                "goal_position": self._goal_position
            }
        }
        return observations

    # Called before each physics step,
    # for instance we can check here if the task was accomplished by
    # changing the color of the cube once its accomplished
    def pre_step(self, control_index, simulation_time):
        cube_position, _ = self._cubeA.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(self._goal_position - cube_position)) < 0.002:
            # Visual Materials are applied by default to the cube
            # in this case the cube has a visual material of type
            # PreviewSurface, we can set its color once the target is reached.
            self._cubeA.get_applied_visual_material().set_color(color=np.array([0, 1.0, 0]))
            self._task_achieved = True
        return

    # Called after each reset,
    # for instance we can always set the gripper to be opened at the beginning after each reset
    # also we can set the cube's color to be blue
    def post_reset(self):
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        self._cubeA.get_applied_visual_material().set_color(color=np.array([0, 0, 1.0]))
        self._task_achieved = False
        return


class Warehouse(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        # We add the task to the world here
        world.add_task(FrankaPlaying(name="my_first_task"))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        # The world already called the setup_scene from the task (with first reset of the world)
        # so we can retrieve the task objects
        self._franka = self._world.scene.get_object("fancy_franka")
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    async def setup_post_reset(self):
        self._controller.reset()
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        # Gets all the tasks observations
        current_observations = self._world.get_observations()
        actions = self._controller.forward(
            picking_position=current_observations["cubeA"]["position"],
            placing_position=current_observations["cubeA"]["goal_position"],
            current_joint_positions=current_observations["fancy_franka"]["joint_positions"],
        )
        self._franka.apply_action(actions)
        if self._controller.is_done():
            self._world.pause()
        return