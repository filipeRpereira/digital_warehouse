from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.tasks import BaseTask, FollowTarget

import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims

from .path_planning_task import FrankaPathPlanningTask

import numpy as np

from omni.isaac.sensor import _sensor
import asyncio
import weakref
import carb
import omni
import asyncio
import weakref
import omni.physx as _physx
import omni.ui as ui
from omni.isaac.sensor import _sensor
import omni.kit.commands
from pxr import Gf, UsdGeom

from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, LABEL_WIDTH
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.core.utils.nucleus import get_assets_root_path

import time

FRANKA_STAGE_PATH = "/Franka"


class FrankaPlaying(BaseTask):
    #NOTE: we only cover here a subset of the task functions that are available,
    # checkout the base class for all the available functions to override.
    # ex: calculate_metrics, is_done..etc.
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._goal_position = np.array([-0.30, -0.50, 0.20])
        self._task_achieved = False
        self.create_ros_action_graph(FRANKA_STAGE_PATH)
        return


    # Here we setup all the assets that we care about in this task.
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        
        self._table = scene.add(FixedCuboid(prim_path="/World/table",
                                            name="table",
                                            position=np.array([-0.075, 0.0, 0.05]),
                                            scale=np.array([1.5, 1.5, 0.1]),
                                            color=np.array([0.5, 0.1, 0.1])))
        
        self._franka_base = scene.add(FixedCuboid(prim_path="/World/_franka_base",
                                            name="_franka_base",
                                            position=np.array([-0.48, 0.0, 0.15]),
                                            scale=np.array([0.23, 0.2, 0.125]),
                                            color=np.array([0.2, 0.4, 0.8])))
        
        self._cubeA = scene.add(DynamicCuboid(prim_path="/World/cubeA",
                                            name="cubeA",
                                            position=np.array([0.05, 0.5, 0.13]),
                                            scale=np.array([0.05, 0.05, 0.05]),
                                            color=np.array([0, 0, 1.0])))

        
        self._cubeB = scene.add(DynamicCuboid(prim_path="/World/cubeB",
                                            name="cubeB",
                                            position=np.array([-0.30, -0.5, 0.13]),
                                            scale=np.array([0.07, 0.07, 0.07]),
                                            color=np.array([0, 0.4, 0.1])))
        

        
        scene.add_default_ground_plane()
        self._franka = scene.add(Franka(prim_path="/World/Fancy_Franka",
                                        name="fancy_franka",
                                        position=np.array([-0.45, 0.0, 0.215])))
        

        self.create_sensor()
        return


    def create_sensor(self):
        #self._assets_root_path = get_assets_root_path()
        #if self._assets_root_path is None:
        #    carb.log_error("Could not find Isaac Sim assets folder")
        #    return

        # Add IMU Sensor
        #omni.usd.get_context().open_stage_async(self._assets_root_path + "/Isaac/Robots/Simple/franka.usd")
        #omni.kit.app.get_app().next_update_async()

        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_0",
            parent="World/Fancy_Franka/panda_link0",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_1",
            parent="World/Fancy_Franka/panda_link1",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_2",
            parent="World/Fancy_Franka/panda_link2",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_3",
            parent="World/Fancy_Franka/panda_link3",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_4",
            parent="World/Fancy_Franka/panda_link4",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_5",
            parent="World/Fancy_Franka/panda_link5",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_6",
            parent="World/Fancy_Franka/panda_link6",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_7",
            parent="World/Fancy_Franka/panda_link7",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor_8",
            parent="World/Fancy_Franka/panda_link8",
            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )

        self._events = omni.usd.get_context().get_stage_event_stream()
        #self._stage_event_subscription = self._events.create_subscription_to_pop(
        #    self._on_stage_event, name="IMU Sensor Sample stage Watch"
        #)
    

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


    def create_ros_action_graph(self, franka_stage_path):
        try:
            og.Controller.edit(
                {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        #("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
                        ("OnTick",              "omni.graph.action.OnTick"),
                        ("ReadSimTime",       "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                        
                        ("isaac_read_imu_node_00",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_01",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_02",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_03",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_04",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_05",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_06",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_07",  "omni.isaac.sensor.IsaacReadIMU"),
                        ("isaac_read_imu_node_08",  "omni.isaac.sensor.IsaacReadIMU"),

                        ("ros1_publish_imu_00",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_01",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_02",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_03",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_04",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_05",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_06",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_07",        "omni.isaac.ros_bridge.ROS1PublishImu"),
                        ("ros1_publish_imu_08",        "omni.isaac.ros_bridge.ROS1PublishImu"),

                        ("SubscribeJointState",     "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
                        ("ArticulationController",  "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick",                 "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",  "PublishJointState.inputs:timeStamp"),

                        ("OnTick.outputs:tick",                         "SubscribeJointState.inputs:execIn"),
                        ("SubscribeJointState.outputs:execOut",         "ArticulationController.inputs:execIn"),
                        ("SubscribeJointState.outputs:jointNames",      "ArticulationController.inputs:jointNames"),
                        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                       
                        ("OnTick.outputs:tick", "isaac_read_imu_node_00.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_01.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_02.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_03.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_04.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_05.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_06.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_07.inputs:execIn"),
                        ("OnTick.outputs:tick", "isaac_read_imu_node_08.inputs:execIn"),

                        ("isaac_read_imu_node_00.outputs:angVel",       "ros1_publish_imu_00.inputs:angularVelocity"),
                        ("isaac_read_imu_node_00.outputs:linAcc",       "ros1_publish_imu_00.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_00.outputs:orientation",  "ros1_publish_imu_00.inputs:orientation"),
                        ("isaac_read_imu_node_00.outputs:execOut",      "ros1_publish_imu_00.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_00.inputs:timeStamp"),

                        ("isaac_read_imu_node_01.outputs:angVel",       "ros1_publish_imu_01.inputs:angularVelocity"),
                        ("isaac_read_imu_node_01.outputs:linAcc",       "ros1_publish_imu_01.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_01.outputs:orientation",  "ros1_publish_imu_01.inputs:orientation"),
                        ("isaac_read_imu_node_01.outputs:execOut",      "ros1_publish_imu_01.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_01.inputs:timeStamp"),

                        ("isaac_read_imu_node_02.outputs:angVel",       "ros1_publish_imu_02.inputs:angularVelocity"),
                        ("isaac_read_imu_node_02.outputs:linAcc",       "ros1_publish_imu_02.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_02.outputs:orientation",  "ros1_publish_imu_02.inputs:orientation"),
                        ("isaac_read_imu_node_02.outputs:execOut",      "ros1_publish_imu_02.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_02.inputs:timeStamp"),

                        ("isaac_read_imu_node_03.outputs:angVel",       "ros1_publish_imu_03.inputs:angularVelocity"),
                        ("isaac_read_imu_node_03.outputs:linAcc",       "ros1_publish_imu_03.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_03.outputs:orientation",  "ros1_publish_imu_03.inputs:orientation"),
                        ("isaac_read_imu_node_03.outputs:execOut",      "ros1_publish_imu_03.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_03.inputs:timeStamp"),

                        ("isaac_read_imu_node_04.outputs:angVel",       "ros1_publish_imu_04.inputs:angularVelocity"),
                        ("isaac_read_imu_node_04.outputs:linAcc",       "ros1_publish_imu_04.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_04.outputs:orientation",  "ros1_publish_imu_04.inputs:orientation"),
                        ("isaac_read_imu_node_04.outputs:execOut",      "ros1_publish_imu_04.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_04.inputs:timeStamp"),

                        ("isaac_read_imu_node_05.outputs:angVel",       "ros1_publish_imu_05.inputs:angularVelocity"),
                        ("isaac_read_imu_node_05.outputs:linAcc",       "ros1_publish_imu_05.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_05.outputs:orientation",  "ros1_publish_imu_05.inputs:orientation"),
                        ("isaac_read_imu_node_05.outputs:execOut",      "ros1_publish_imu_05.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_05.inputs:timeStamp"),

                        ("isaac_read_imu_node_06.outputs:angVel",       "ros1_publish_imu_06.inputs:angularVelocity"),
                        ("isaac_read_imu_node_06.outputs:linAcc",       "ros1_publish_imu_06.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_06.outputs:orientation",  "ros1_publish_imu_06.inputs:orientation"),
                        ("isaac_read_imu_node_06.outputs:execOut",      "ros1_publish_imu_06.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_06.inputs:timeStamp"),

                        ("isaac_read_imu_node_07.outputs:angVel",       "ros1_publish_imu_07.inputs:angularVelocity"),
                        ("isaac_read_imu_node_07.outputs:linAcc",       "ros1_publish_imu_07.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_07.outputs:orientation",  "ros1_publish_imu_07.inputs:orientation"),
                        ("isaac_read_imu_node_07.outputs:execOut",      "ros1_publish_imu_07.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_07.inputs:timeStamp"),

                        ("isaac_read_imu_node_08.outputs:angVel",       "ros1_publish_imu_08.inputs:angularVelocity"),
                        ("isaac_read_imu_node_08.outputs:linAcc",       "ros1_publish_imu_08.inputs:linearAcceleration"),
                        ("isaac_read_imu_node_08.outputs:orientation",  "ros1_publish_imu_08.inputs:orientation"),
                        ("isaac_read_imu_node_08.outputs:execOut",      "ros1_publish_imu_08.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime",          "ros1_publish_imu_08.inputs:timeStamp"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ros1_publish_imu_00.inputs:topicName", "imu_link0"),
                        ("ros1_publish_imu_01.inputs:topicName", "imu_link1"),
                        ("ros1_publish_imu_02.inputs:topicName", "imu_link2"),
                        ("ros1_publish_imu_03.inputs:topicName", "imu_link3"),
                        ("ros1_publish_imu_04.inputs:topicName", "imu_link4"),
                        ("ros1_publish_imu_05.inputs:topicName", "imu_link5"),
                        ("ros1_publish_imu_06.inputs:topicName", "imu_link6"),
                        ("ros1_publish_imu_07.inputs:topicName", "imu_link7"),
                        ("ros1_publish_imu_08.inputs:topicName", "imu_link8"),

                        ("SubscribeJointState.inputs:topicName", "joint_states_sim"),
                        ("PublishJointState.inputs:topicName",   "joint_states_sim"),

                        ("OnTick.inputs:framePeriod", 0),
                        ("ReadSimTime.inputs:resetOnStop", True),
                    ],
                },
            )
             # Setting the /panda target prim to Publish JointState node
            set_target_prims(primPath="/World/ActionGraph/PublishJointState", 
                             targetPrimPaths=["/World/Fancy_Franka"])    
        
        
            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_00", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link0/imu_sensor_0"], 
                             inputName= "inputs:imuPrim") 

            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_01", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link1/imu_sensor_1"], 
                             inputName= "inputs:imuPrim")            

            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_02", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link2/imu_sensor_2"], 
                             inputName= "inputs:imuPrim") 
            
            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_03", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link3/imu_sensor_3"], 
                             inputName= "inputs:imuPrim") 
            
            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_04", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link4/imu_sensor_4"], 
                             inputName= "inputs:imuPrim") 
            
            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_05", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link5/imu_sensor_5"], 
                             inputName= "inputs:imuPrim") 
            
            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_06", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link6/imu_sensor_6"], 
                             inputName= "inputs:imuPrim") 
            
            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_07", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link7/imu_sensor_7"], 
                             inputName= "inputs:imuPrim") 
            
            set_target_prims(primPath="/World/ActionGraph/isaac_read_imu_node_08", 
                             targetPrimPaths=["/World/Fancy_Franka/panda_link8/imu_sensor_8"], 
                             inputName= "inputs:imuPrim") 

        except Exception as e:
            print(e)


class WarehouseTask3Manual(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        # We add the task to the world here
        world.add_task(FrankaPlaying(name="task_3"))
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
            end_effector_initial_height=0.30,
            #events_dt = [0.008, 0.008, 1, 1.2, 0.10, 0.10, 0.0025, 1, 0.008, 0.1]
            events_dt = [0.008, 0.008, 1, 1.0, 1.0, 1.0, 0.0025, 1, 0.008, 0.1]
           
        )

        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        #await self._world.play_async()
        return
    
    async def _on_stacking_event_async(self):
        world = self.get_world()
        #self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await world.play_async()
        return

    async def setup_pre_reset(self):
        world = self.get_world()
        #if world.physics_callback_exists("sim_step"):
        #    world.remove_physics_callback("sim_step")
        self._controller.reset()
        
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
            f = open("demofile2.txt", "w")
            f.write(str(current_observations["fancy_franka"]["joint_positions"]))
            f.close()

            #print(current_observations["fancy_franka"]["joint_positions"])
            self._world.pause()
        return
    

