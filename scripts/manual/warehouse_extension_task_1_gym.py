# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.user_examples import WarehouseTask1Gym

import asyncio
import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder


class WarehouseExtensionTask1Gym(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="WareHouse",
            submenu_name="Tasks 1",
            name="Isaac Gym",
            title="Warehouse project",
            doc_link="",
            overview="An warehouse project.",
            file_path=os.path.abspath(__file__),
            sample=WarehouseTask1Gym(),
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        return
    
    def _on_stacking_button_event(self):
        asyncio.ensure_future(self.sample._on_stacking_event_async())
        self.task_ui_elements["Task 1"].enabled = False
        return

    def post_reset_button_event(self):
        self.task_ui_elements["Task 1"].enabled = True
        return

    def post_load_button_event(self):
        self.task_ui_elements["Task 1"].enabled = True
        return

    def post_clear_button_event(self):
        self.task_ui_elements["Task 1"].enabled = False
        return
    
    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True
                dict = {
                    "label": "Start Stacking",
                    "type": "button",
                    "text": "Start listening ROS",
                    "tooltip": "Task 1",
                    "on_clicked_fn": self._on_stacking_button_event,
                }

                self.task_ui_elements["Task 1"] = btn_builder(**dict)
                self.task_ui_elements["Task 1"].enabled = False
                
