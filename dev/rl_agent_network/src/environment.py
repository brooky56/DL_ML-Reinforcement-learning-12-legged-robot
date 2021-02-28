import os
import logging
import numpy as np

from gym import Space
from gym.spaces.box import Box
from gym.spaces.dict import Dict
from pyrep import PyRep, objects

from catalyst_rl.rl.core import EnvironmentSpec
from catalyst_rl.rl.utils import extend_space


class CoppeliaSimEnvWrapper(EnvironmentSpec):
    def __init__(self, visualize=True,
                 mode="train",
                 **params):
        super().__init__(visualize=visualize, mode=mode)

        # Scene selection
        scene_file_path = os.path.join(os.getcwd(), 'scene/strirus.ttt')

        # Simulator launch
        self.env = PyRep()
        self.env.launch(scene_file_path, headless=True)
        self.env.start()
        self.env.step()

        # Task related initialisations in Simulator
        self.robot = objects.dummy.Dummy("Strirus")

        self.step_counter = 0
        self.max_step_count = 100
        self.target_pose = None
        self.initial_distance = None
        self.image_width, self.image_height = 320, 240
        self.vision_sensor.set_resolution((self.image_width, self.image_height))
        self._history_len = 1

    # All methods are required for Gym environment

    @property
    def history_len(self):
        return self._history_len

    @property
    def observation_space(self) -> Space:
        return self._observation_space

    @property
    def state_space(self) -> Space:
        return self._state_space

    @property
    def action_space(self) -> Space:
        return self._action_space

    def step(self, action):
        done = False
        info = {}
        prev_distance_to_goal = self.distance_to_goal()

        # Make a step in simulation
        self.apply_controls(action)
        self.env.step()
        self.step_counter += 1

        # Reward calculations
        success_reward = self.success_check()
        distance_reward = (prev_distance_to_goal - self.distance_to_goal()) / self.initial_distance

        reward = distance_reward + success_reward

        # Check reset conditions
        if self.step_counter > self.max_step_count:
            done = True
            logging.info('--------Reset: Timeout--------')
        elif self.distance_to_goal() > 0.8:
            done = True
            logging.info('--------Reset: Too far from target--------')
        elif self.collision_check():
            done = True
            logging.info('--------Reset: Collision--------')

        return self.get_observation(), reward, done, info

    def reset(self):
        logging.info("Episode reset...")
        self.step_counter = 0
        self.env.stop()
        self.env.start()
        self.env.step()
        self.setup_scene()
        observation = self.get_observation()
        return observation

    def distance_to_goal(self):
        # Calculate distance to goal (difference between current position and goal position)
        return []

    def setup_goal(self):
        # Set goal position
        return []

    def setup_scene(self):
        self.setup_goal()
        # set initial positions
        return []

    def get_observation(self):
        joint_obs = []
        return {"joint_obs": joint_obs}

    def collision_check(self):
        return self.grasped_STL.check_collision(
            self.stacking_area) or self.grasped_STL.check_collision(self.goal_STL)

    def success_check(self):
        success_reward = 0.
        if self.distance_to_goal() < 0.01:
            success_reward = 1
            logging.info('--------Success state--------')
        return success_reward

    def apply_controls(self, action):
        # set new position
        return []
