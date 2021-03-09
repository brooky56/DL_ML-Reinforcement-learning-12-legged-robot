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
        self.imu_sensor = objects.sensor.Sensor("imu_sensor")
        self.robot_ghost = objects.dummy.Dummy("goal_target")
        self.robot_ghost_zero_pose = self.robot_ghost.get_pose()
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
        goal_pos = self.goal.get_position()
        robot_pos = self.robot_ghost.get_position()
        return np.linalg.norm(np.array(robot_pos) - np.array(goal_pos))

    def setup_goal(self):
        # Set goal position
        goal_position = self.robot_ghost
        # 2D goal randomization
        self.target_pose = [goal_position[0] + (2 * np.random.rand() - 1.) * 0.1,
                            goal_position[1] + (2 * np.random.rand() - 1.) * 0.1,
                            goal_position[2]]
        self.goal.set_pose(self.target_pose)

        # Randomizing the RGB of the goal and the plane
        rgb_values_goal = list(np.random.rand(3,))
        rgb_values_plane = list(np.random.rand(3,))
        self.stacking_area.set_color(rgb_values_plane)
        self.initial_distance = self.distance_to_goal()


    def setup_scene(self):
        self.setup_goal()
        self.robot_ghost.set_pose(self.robot_ghost_zero_pose)

    def get_observation(self):
        # imu_data = self.imu_sensor.get_data()
        joint_obs = self.handler.joints_state_space.get_data()
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
        robot_position = self.robot.get_position()
        new_position = [robot_position[i] + (action[i] / 200.) for i in range(3)]
        self.robot.set_position(new_position)
