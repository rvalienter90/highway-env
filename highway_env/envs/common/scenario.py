import numpy as np
from highway_env.road.road import Road, RoadNetwork
from highway_env.vehicle.controller import ControlledVehicle
from highway_env.vehicle.kinematics import Vehicle
from highway_env.road.lane import LineType, StraightLane, SineLane, CircularLane, AbstractLane
from highway_env.road.regulation import RegulatedRoad
from highway_env.vehicle.objects import Obstacle
from highway_env.vehicle.behavior import CustomVehicle
from highway_env import utils
from highway_env.road.lane import CircularLane
from highway_env.utils import near_split
from gym.utils import seeding
import random
import copy


class Scenario:
    def __init__(self, env, scenario_number=0,complex=False,simple=False):
        self.env = env
        self.env.default_config = copy.deepcopy(env.config)
        self.road = None
        self.controlled_vehicles = None
        # self.road_types = ["intersection", "roundabout", "highway","twoway","uturn","road_merge","road_exit"]
        self.road_types = self.env.config['scenario']['road_types']
        # self.road_types = ["road_exit"]
        # self.complex = self.env.config['scenario']['complex']
        # self.simple = self.env.config['scenario']['simple']
        self.complex = False
        self.simple = False
        self.road_types_idx = -1
        # self.road_missions = ["merging","exit"]
        if scenario_number != 0:
            if scenario_number == 2:
                self.env.config.update(self.default_config_merge())
            if scenario_number == 3:
                self.env.config.update(self.default_config_exit())

        self.random_scenario = self.env.config['scenario']['random_scenario']
        if self.random_scenario:
            # idx = np.random.randint(0, len(self.road_types))
            self.road_types_idx = idx =self.env.episode%len(self.road_types)
            self.road_type = self.road_types[idx]
            self.env.config['scenario']['road_type'] = self.road_type
            if self.road_type == "road_merge":
                self.mission_type ="merging"
                self.env.config['screen_width'] = 2900
                self.env.config['screen_height'] = 300
            elif self.road_type == "road_exit":
                self.mission_type = "exit"
                self.env.config['screen_width'] = 2900
                self.env.config['screen_height'] = 300
            elif self.road_type == "intersection":
                self.env.config['screen_width'] =900
                self.env.config['screen_height'] = 900
                self.env.config['controlled_vehicle']['controlled_vehicle_speed'] = 15
                self.mission_type = "none"
            elif self.road_type == "roundabout":
                self.env.config['screen_width'] = 900
                self.env.config['screen_height'] = 900
                self.mission_type = "none"
            elif self.road_type == "uturn":
                self.env.config['screen_width'] = 1000
                self.env.config['screen_height'] = 500
                self.mission_type = "none"
            else:
                self.env.config['screen_width'] = 2900
                self.env.config['screen_height'] = 300
                self.mission_type = "none"

            self.env.config['scenario']['mission_type'] = self.mission_type
        else:
            self.road_type = self.env.config['scenario']['road_type']


        random_offset = copy.deepcopy(self.env.config['scenario']['random_offset'])
        delta_before, delta_converging, delta_merge = (0, 0, 0)
        if self.env.config['scenario']['randomize_before']:
            delta_before = np.random.randint(low=random_offset[0], high=random_offset[1])
        if self.env.config['scenario']['randomize_converging']:
            delta_converging = np.random.randint(low=random_offset[0], high=random_offset[1])
        if self.env.config['scenario']['randomize_merge']:
            delta_merge = np.random.randint(low=random_offset[0], high=random_offset[1])

        self.before_merging = self.env.config['scenario']['before_merging'] + delta_before
        self.converging_merging = self.env.config['scenario']['converging_merging'] + delta_converging
        self.during_merging = self.env.config['scenario']['during_merging'] + delta_merge
        self.after_merging = self.env.config['scenario']['after_merging']



        self.randomize_vehicles = self.env.config['scenario']['randomize_vehicles']
        self.random_offset_vehicles = copy.deepcopy(self.env.config['scenario']['random_offset_vehicles'])
        self.randomize_speed = self.env.config['scenario']['randomize_speed']
        self.randomize_speed_offset = copy.deepcopy(self.env.config['scenario']['randomize_speed_offset'])
        self.controlled_vehicles_count = self.env.config['controlled_vehicles']


        self.random_controlled_vehicle = self.env.config['scenario']['random_controlled_vehicle']
        # if self.env.config['scenario']['randomize_vehicles']:
        #     self.cruising_vehicles_count_rightmost_lane = self.env.config['vehicles_in_rightmost_lane'] - 1
        #     self.cruising_vehicles_count_other_lanes = self.env.config['vehicles_in_other_lanes']
        # else:
        #     self.cruising_vehicles_count_rightmost_lane = self.env.config['vehicles_count'] - 1

        self.cruising_vehicles_count = self.env.config['vehicles_count'] - 1
        self.cruising_vehicles_front_count = self.env.config['cruising_vehicles_front_count']
        self.cruising_vehicles_front = self.env.config['cruising_vehicles_front']
        self.cruising_vehicles_front_random_everywhere = self.env.config['cruising_vehicles_front_random_everywhere']
        self.cruising_vehicles_front_initial_position = self.env.config['cruising_vehicles_front_initial_position']
        self.total_number_of_vehicles = self.env.config['scenario']['total_number_of_vehicles']
        self.prob_of_controlled_vehicle = self.env.config['scenario']['prob_of_controlled_vehicle']
        self.controlled_baseline_vehicle = self.env.config['controlled_baseline_vehicle']

        # self.np_random, seed = seeding.np_random(seed)


        if self.env.config['scenario']['random_lane_count']:
            lane_interval = copy.deepcopy(self.env.config['scenario']['lane_count_interval'])
            self.lanes_count = np.random.randint(low=lane_interval[0], high=lane_interval[1])
        else:
            self.lanes_count = self.env.config['lanes_count']

        # self.cruising_vehicle = copy.deepcopy({"vehicles_type": self.env.config['cruising_vehicle']["vehicles_type"],
        #                          "speed": self.env.config['cruising_vehicle']["speed"],
        #                          "enable_lane_change": self.env.config['cruising_vehicle']['enable_lane_change'],
        #                          'length': self.env.config['cruising_vehicle']['length']
        #                          })
        self.cruising_vehicle = copy.deepcopy(self.env.config['cruising_vehicle'])
        self.merging_vehicle = copy.deepcopy(self.env.config['merging_vehicle'])
        self.baseline_vehicle = copy.deepcopy(self.env.config['baseline_vehicle'])
        self.controlled_vehicle = copy.deepcopy(self.env.config['controlled_vehicle'])
        # self.controlled_vehicle_speed = self.env.config['scenario']['controlled_vehicle_speed']
        self.controlled_vehicle_speed = self.controlled_vehicle['controlled_vehicle_speed']

        # self.merging_vehicle = copy.deepcopy({'id': self.env.config['merging_vehicle']['id'],
        #                         'speed': self.env.config['merging_vehicle']['speed'],
        #                         'initial_position': self.env.config['merging_vehicle']['initial_position'],
        #                         'random_offset_merging': self.env.config['merging_vehicle']['random_offset_merging'],
        #                         'controlled_vehicle': self.env.config['merging_vehicle']['controlled_vehicle'],
        #                         'vehicles_type': self.env.config['merging_vehicle']["vehicles_type"],
        #                         'set_route': self.env.config['merging_vehicle']['set_route'],
        #                         'randomize': self.env.config['merging_vehicle']['randomize'],
        #                         'randomize_speed_merging': self.env.config['merging_vehicle']['randomize_speed_merging'],
        #                         'min_speed': self.env.config['merging_vehicle']['min_speed'],
        #                         'max_speed': self.env.config['merging_vehicle'][ 'max_speed'],
        #                         })
        # #
        self.exit_vehicle = copy.deepcopy({'id': self.env.config['exit_vehicle']['id'],
                             'speed': self.env.config['exit_vehicle']['speed'],
                             'initial_position': self.env.config['exit_vehicle']['initial_position'],
                             'controlled_vehicle': self.env.config['exit_vehicle']['controlled_vehicle'],
                             'vehicles_type': self.env.config['exit_vehicle']["vehicles_type"],
                             'set_route': self.env.config['exit_vehicle']['set_route'],
                                           'random_offset_exit': self.env.config['exit_vehicle']['random_offset_exit'],
                             'randomize': self.env.config['exit_vehicle']['randomize']
                             })

        self.other_vehicles_type = self.env.config["other_vehicles_type"]
        self.record_history = self.env.config["show_trajectories"]
        self.ego_spacing = self.env.config["ego_spacing"]
        self.initial_lane_id = self.env.config["initial_lane_id"]
        self.vehicles_density = self.env.config["vehicles_density"]

        self.scenario_config = copy.deepcopy(self.env.config['scenario'])
        # self.before_exit = self.env.config['scenario']['before_exit']
        # self.converging_exit = self.env.config['scenario']['converging_exit']
        # self.taking_exit = self.env.config['scenario']['taking_exit']
        # self.during_exit = self.env.config['scenario']['during_exit']
        # self.after_exit = self.env.config['scenario']['after_merging']

        self.exit_humans = self.env.config['scenario']['exit_humans']
        self.exit_controlled = self.env.config['scenario']['exit_controlled']
        self.exit_length = self.env.config['scenario']['exit_length']
        self.after_exit = self.env.config['scenario']['after_exit']
        self.simulation_frequency = self.env.config["simulation_frequency"]
        self.np_random = self.env.np_random

        self._create_road(self.road_type)
        self._create_vehicles(self.road_type)

    def create_random(self,cruising_vehicle_class,  from_options =None, speed: float = None, lane_id = None, spacing: float = 1, initial_possition = None, enable_lane_change = True, vehicle_id = 0 , right_lane = None) \
            -> "Vehicle":
        """
        Create a random vehicle on the road.

        The lane and /or speed are chosen randomly, while longitudinal position is chosen behind the last
        vehicle in the road with density based on the number of lanes.

        :param road: the road where the vehicle is driving
        :param speed: initial speed in [m/s]. If None, will be chosen randomly
        :param lane_id: id of the lane to spawn in
        :param spacing: ratio of spacing to the front vehicle, 1 being the default
        :return: A vehicle with random position and/or speed
        """
        if speed is None:
            speed = self.road.np_random.uniform(Vehicle.DEFAULT_SPEEDS[0], Vehicle.DEFAULT_SPEEDS[1])
        default_spacing = 1.5 * speed
        if from_options is None:
            _from = self.road.np_random.choice(list(self.road.network.graph.keys()))
        else:
            _from = self.road.np_random.choice(from_options)

        _to = self.road.np_random.choice(list(self.road.network.graph[_from].keys()))
        if _from == "a" or _from == "b":
            lanes_count = len(self.road.network.graph[_from][_to]) -1
        else:
            lanes_count = len(self.road.network.graph[_from][_to])

        _id = lane_id if lane_id is not None else self.road.np_random.choice(lanes_count)


        # if right_lane:
        #     _id = min(_id, right_lane)
        lane = self.road.network.get_lane((_from, _to, _id))
        offset = spacing * default_spacing * np.exp(-5 / 30 * len(self.road.network.graph[_from][_to]))
        if initial_possition:
            x0 = initial_possition
        else:
            # x0 = np.max([lane.local_coordinates(v.position)[0] for v in self.road.vehicles]) \
            #     if len(self.road.vehicles) else 3 * offset
            distances = []
            for v in self.road.vehicles:
                test = v.lane_index[2]
                if v.lane_index[2] <= lanes_count - 1:
                    distances.append(lane.local_coordinates(v.position)[0])

            x0 = np.max([distances]) if distances else 3 * offset

        x0 += offset * self.road.np_random.uniform(0.9, 1.1)
        x0 = max(0, x0)
        vehicle = cruising_vehicle_class(self.road,
                                         lane.position(x0, 0),
                                         speed=speed, enable_lane_change=enable_lane_change,
                                         config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)

        # v = cls(road, lane.position(x0, 0), lane.heading_at(x0), speed)
        return vehicle

    def default_config_merge(self) -> dict:
        """
        :return: a configuration dict
        """
        return {

            'duration': 15,  # 40

            'scenario': {
                'scenario_number': 2,
                'road_type': "road_merge",
                # 1-highway, 2-road_closed , 3-road_merge , 4-road_exit, Road types should match with is vehicle_type 1,2,3

                # for merging road
                'lane_count_interval': [1, 4],  # random number of lane range
                'random_offset': [-5, 5],  # offset values for before, converging, merge  -+
                'before_merging': 100,
                'randomize_before': False,  # random before road size
                # distance before converging, converging is the start of the lane with slope
                'converging_merging': 200,
                'randomize_converging': False,  # random converging road size
                # distance from converging to merge, merge start when the slope lane ends
                'during_merging': 110,  # distance of the merging lane, paralles to highway
                'randomize_merge': False,  # random merge road size
                'random_lane_count': False,  # random number of lane
                'after_merging': 1100,  # distance of the highway after that

                # for exit road
                'before_exit': 100,
                'converging_exit': 50,
                'taking_exit': 80,
                'during_exit': 100,
                'after_exit': 1100,

                'randomize_vehicles': True,  # if true vehicles will be randomize based on random_offset_vehicles values
                'random_offset_vehicles': [-5, 5],
                # 'vehicles_in_rightmost_lane':10,  # will overide_vehicle_count if randomize_vehicles = True
                # 'vehicles_in_other_lanes':10,
                'random_controlled_vehicle': False,
                # will chose controlled_vehicle  based on prob_of_controlled_vehicle, override controlled_vehicle
                'total_number_of_vehicles': 13,
                # will be the total number of vehicles in the scenario, AV or cruising will be chosen based on the prob, overide vehicle_count
                'prob_of_controlled_vehicle': 0.5,
                'mission_type': 'merging',

                # if shuffle_controlled_vehicle , from total_number_of_vehicles with probability prob_of_controlled_vehicle AV willl be chosen

            },
            # 'cruising_vehicle': {
            #     'acc_max': 6,  # """Maximum acceleration."""
            #     'comfort_acc_max': 4,  # """Desired maximum acceleration."""
            #     'comfort_acc_min': -12,  # """Desired maximum deceleration."""
            #     'distance_wanted': 0.51,  # """Desired jam distance to the front vehicle."""
            #     'time_wanted': 0.5,  # """Desired time gap to the front vehicle."""
            #     'delta': 4,  # """Exponent of the velocity term."""
            #     'speed': 25,  # Vehicle speed
            #     'enable_lane_change': False,  # allow lane change
            #
            #     'vehicles_type': "highway_env.vehicle.behavior.CustomVehicle",
            #     # chose different vehicle types from :
            #     # "highway_env.vehicle.behavior.CustomVehicle" ,"highway_env.vehicle.behavior.AggressiveVehicle","highway_env.vehicle.behavior.DefensiveVehicle", "highway_env.vehicle.behavior.LinearVehicle"  "highway_env.vehicle.behavior.IDMVehicle"
            #     # if CustomVehicle is chosen it will load the previous configurations, other vehicles types has their own predefiened configurations.
            #     'length': 5.0,  # Vehicle length [m]
            #     'width': 2.0,  # Vehicle width [m]
            #     'max_speed': 40  # Maximum reachable speed [m/s]
            # },

            'merging_vehicle': {
                'acc_max': 6,  # """Maximum acceleration.""" 6
                'comfort_acc_max': 3,  # """Desired maximum acceleration.""" 3
                'comfort_acc_min': -5,  # """Desired maximum deceleration.""" -5
                'distance_wanted': 0.5,  # """Desired jam distance to the front vehicle.""" 5
                'time_wanted': 0.5,  # """Desired time gap to the front vehicle.""" 1.5
                'delta': 4,  # """Exponent of the velocity term.""" 4
                'speed': 25,
                'initial_position': [78, 0],
                'enable_lane_change': False,
                'controlled_vehicle': False,  # chose if merging vehicle is AV or human
                'vehicles_type': "highway_env.vehicle.behavior.CustomVehicle",
                'set_route': False,  # predefine the route
                # "highway_env.vehicle.behavior.CustomVehicle" ,"highway_env.vehicle.behavior.AggressiveVehicle","highway_env.vehicle.behavior.DefensiveVehicle", "highway_env.vehicle.behavior.LinearVehicle"  "highway_env.vehicle.behavior.IDMVehicle"
                # if CustomVehicle is chosen it will load the previous configurations, other vehicles types has their own predefiened configurations.
                'randomize': True,
                'id': -1,  # id for the merging vehicle

                'length': 5.0,  # Vehicle length [m]
                'width': 2.0,  # Vehicle width [m]
                'max_speed': 40  # Maximum reachable speed [m/s]

            },

            "reward": {
                "coop_reward_type": "multi_agent_tuple",
                "reward_type": "merging_reward",  # merging_reward
                "normalize_reward": True,
                "reward_speed_range": [20, 40],
                "collision_reward": -2,  # -1
                "on_desired_lane_reward": 0.3,
                "high_speed_reward": 0.6,  # 0.4
                "lane_change_reward": -0.2,
                "target_lane": 1,
                "distance_reward": -0.1,
                "distance_merged_vehicle_reward": 0,
                "distance_reward_type": "min",
                "successful_merging_reward": 5,
                "continuous_mission_reward": True,
                "cooperative_flag": True,
                "sympathy_flag": True,
                "cooperative_reward": 0.9,
                # True : after merging will keep receiving the reward, False: just received the reward once

            }

        }

    def default_config_exit(self) -> dict:
        """
        :return: a configuration dict
        """
        return {

            'scenario': {
                'scenario_number': 3,
                'road_type': "road_exit",
                # 1-highway, 2-road_closed , 3-road_merge , 4-road_exit, 5-test Road types should match with is vehicle_type 1,2,3

                # for merging road
                'lane_count_interval': [1, 4],  # random number of lane range
                'random_offset': [-5, 5],  # offset values for before, converging, merge  -+
                'before_merging': 100,
                'randomize_before': False,  # random before road size
                # distance before converging, converging is the start of the lane with slope
                'converging_merging': 200,
                'randomize_converging': False,  # random converging road size
                # distance from converging to merge, merge start when the slope lane ends
                'during_merging': 110,  # distance of the merging lane, paralles to highway
                'randomize_merge': False,  # random merge road size
                'random_lane_count': False,  # random number of lane
                'after_merging': 1100,  # distance of the highway after that

                # for exit road
                'before_exit': 100,
                'converging_exit': 50,
                'taking_exit': 40,
                'during_exit': 100,
                'after_exit': 1100,

                'randomize_vehicles': True,  # if true vehicles will be randomize based on random_offset_vehicles values
                'random_offset_vehicles': [-5, 5],
                'random_controlled_vehicle': False,
                # will chose controlled_vehicle  based on prob_of_controlled_vehicle, override controlled_vehicle
                'total_number_of_vehicles': 13,
                # will be the total number of vehicles in the scenario, AV or cruising will be chosen based on the prob, overide vehicle_count
                'prob_of_controlled_vehicle': 0.5,
                'mission_type': 'exit',

                # if shuffle_controlled_vehicle , from total_number_of_vehicles with probability prob_of_controlled_vehicle AV willl be chosen

            },

            # 'cruising_vehicle': {
            #     'acc_max': 6,  # """Maximum acceleration."""
            #     'comfort_acc_max': 4,  # """Desired maximum acceleration."""
            #     'comfort_acc_min': -12,  # """Desired maximum deceleration."""
            #     'distance_wanted': 0.51,  # """Desired jam distance to the front vehicle."""
            #     'time_wanted': 0.5,  # """Desired time gap to the front vehicle."""
            #     'delta': 4,  # """Exponent of the velocity term."""
            #     'speed': 25,  # Vehicle speed
            #     'enable_lane_change': False,  # allow lane change
            #
            #     'vehicles_type': "highway_env.vehicle.behavior.CustomVehicle",
            #     # chose different vehicle types from :
            #     # "highway_env.vehicle.behavior.CustomVehicle" ,"highway_env.vehicle.behavior.AggressiveVehicle","highway_env.vehicle.behavior.DefensiveVehicle", "highway_env.vehicle.behavior.LinearVehicle"  "highway_env.vehicle.behavior.IDMVehicle"
            #     # if CustomVehicle is chosen it will load the previous configurations, other vehicles types has their own predefiened configurations.
            #
            #     'length': 5.0,  # Vehicle length [m]
            #     'width': 2.0,  # Vehicle width [m]
            #     'max_speed': 40  # Maximum reachable speed [m/s]
            # },

            'exit_vehicle': {
                'acc_max': 6,  # """Maximum acceleration.""" 6
                'comfort_acc_max': 3,  # """Desired maximum acceleration.""" 3
                'comfort_acc_min': -5,  # """Desired maximum deceleration.""" -5
                'distance_wanted': 0.5,  # """Desired jam distance to the front vehicle.""" 5
                'time_wanted': 0.5,  # """Desired time gap to the front vehicle.""" 1.5
                'delta': 4,  # """Exponent of the velocity term.""" 4
                'speed': 25,
                'initial_position': [78, 0],
                'enable_lane_change': True,
                'controlled_vehicle': False,  # chose if merging vehicle is AV or human
                'vehicles_type': "highway_env.vehicle.behavior.CustomVehicle",
                'set_route': True,  # predefine the route
                # "highway_env.vehicle.behavior.CustomVehicle" ,"highway_env.vehicle.behavior.AggressiveVehicle","highway_env.vehicle.behavior.DefensiveVehicle", "highway_env.vehicle.behavior.LinearVehicle"  "highway_env.vehicle.behavior.IDMVehicle"
                # if CustomVehicle is chosen it will load the previous configurations, other vehicles types has their own predefiened configurations.
                'randomize': True,
                'id': -1,  # id for the merging vehicle

                'length': 5.0,  # Vehicle length [m]
                'width': 2.0,  # Vehicle width [m]
                'max_speed': 40  # Maximum reachable speed [m/s]
            },

            "reward": {
                "coop_reward_type": "multi_agent_tuple",
                "reward_type": "exit_reward",  # merging_reward
                "normalize_reward": True,
                "reward_speed_range": [20, 40],
                "collision_reward": -2,  # -1
                "on_desired_lane_reward": 0.3,
                "high_speed_reward": 0.6,  # 0.4
                "lane_change_reward": -0.2,
                "target_lane": 1,
                "distance_reward": -0.1,
                "distance_merged_vehicle_reward": 0,
                "distance_reward_type": "min",
                "successful_merging_reward": 5,
                "continuous_mission_reward": True,
                "cooperative_flag": True,
                "sympathy_flag": True,
                "cooperative_reward": 0.9,
                # True : after merging will keep receiving the reward, False: just received the reward once

            }

        }

    def _create_road(self, road_type) -> None:
        if road_type == "highway":
            self._road_highway()
        elif road_type == "road_merge":
            self._road_merge()

        elif road_type == "road_exit":
            self._road_exit()
        elif road_type == "intersection":
            self._road_intersection()
        elif road_type == "roundabout":
            self._road_roundabout()
        elif road_type == "twoway":
            self._road_twoway()
        elif road_type == "uturn":
            self._road_uturn()
        elif road_type == "road_closed":
            # TODO , fix arguments
            self._road_closed(end=self.before_merging + self.converging_merging, after=self.after_merging)
        elif road_type == "test":
            self._road_test()

    def _create_vehicles(self, road_type):
        if road_type == "road_merge":
            if self.random_controlled_vehicle:
                self._vehicles_merge_to_highway_prob()
            else:
                self._vehicles_merge_to_highway()
        elif road_type == "road_exit":
            self._vehicles_exit_highway()
        elif road_type == "intersection":
            self._vehicles_intersection()
        elif road_type == "roundabout":
            self._vehicles_roundabout()
        elif road_type == "road_closed":
            # TODO , fix arguments
            self._vehicles_road_closed(controlled_vehicles=self.controlled_vehicles,
                                       cruising_vehicles_count=self.cruising_vehicles_count)
        elif road_type == "highway":
            self._vehicles_highway()
        elif road_type == "twoway":
            self._vehicles_twoway()
        elif road_type == "uturn":
            self._vehicles_uturn()
        elif road_type == "test":
            self._vehicle_road_test()

    def _road_merge(self):
        """Create a road composed of straight adjacent lanes."""

        net = RoadNetwork()

        # Highway lanes
        ends = [self.before_merging, self.converging_merging, self.during_merging,
                self.after_merging]  # Before, converging, merge, after
        c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE

        for lane in range(self.lanes_count):
            line_types = [LineType.CONTINUOUS_LINE if lane == 0 else LineType.STRIPED,
                          LineType.CONTINUOUS_LINE if lane == self.lanes_count - 1 else LineType.NONE]

            net.add_lane("a", "b", StraightLane([0, StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                [sum(ends[:2]), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                line_types=line_types))
            net.add_lane("b", "c",
                         StraightLane([sum(ends[:2]), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                      [sum(ends[:3]), StraightLane.DEFAULT_WIDTH * (lane + 1)], line_types=line_types))
            net.add_lane("c", "d", StraightLane([sum(ends[:3]), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                [sum(ends), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                line_types=line_types))

        # Merging lane
        amplitude = 3.25
        ljk = StraightLane([0, 6.5 + 4 + self.lanes_count * 4], [ends[0], 6.5 + 4 + self.lanes_count * 4],
                           line_types=[c, c],
                           forbidden=True)
        lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
                       amplitude, 2 * np.pi / (2 * ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
        lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
                           line_types=[n, c], forbidden=True)
        net.add_lane("j", "k", ljk)
        net.add_lane("k", "b", lkb)
        net.add_lane("b", "c", lbc)
        road = Road(network=net, np_random=self.env.np_random, record_history=self.record_history)
        road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))

        self.road = road

    def _road_exit1(self):
        """Create a road composed of straight adjacent lanes."""

        net = RoadNetwork()

        # Highway lanes
        ends = [self.before_exit + self.converging_exit, self.taking_exit,
                self.after_exit]  # Before, converging, merge, after
        c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE

        for lane in range(self.lanes_count):
            line_types = [LineType.CONTINUOUS_LINE if lane == 0 else LineType.STRIPED,
                          LineType.CONTINUOUS_LINE if lane == self.lanes_count - 1 else LineType.NONE]

            net.add_lane("a", "b", StraightLane([0, StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                [sum(ends[:1]), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                line_types=line_types))
            net.add_lane("b", "c",
                         StraightLane([sum(ends[:1]), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                      [sum(ends[:2]), StraightLane.DEFAULT_WIDTH * (lane + 1)], line_types=line_types))
            net.add_lane("c", "d", StraightLane([sum(ends[:2]), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                [sum(ends), StraightLane.DEFAULT_WIDTH * (lane + 1)],
                                                line_types=line_types))

        # Exit lane
        amplitude = 3.25 / 4
        lbp = StraightLane([self.before_exit + self.converging_exit, 4 + self.lanes_count * 4],
                           [self.before_exit + self.converging_exit + self.taking_exit, 4 + self.lanes_count * 4],
                           line_types=[n, c], forbidden=True)

        # ljk = StraightLane([0, 6.5 + 4 +self.lanes_count*4], [ends[0], 6.5 + 4 + self.lanes_count*4 ], line_types=[c, c],
        #                    forbidden=True)
        lpk = SineLane(lbp.position(self.taking_exit, amplitude),
                       lbp.position(self.taking_exit + self.during_exit, amplitude),
                       -amplitude, 2 * np.pi / (2 * ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)

        lkj = StraightLane(lpk.position(self.during_exit, 0), lpk.position(self.during_exit + self.after_exit, 0),
                           line_types=[c, c], forbidden=True)

        net.add_lane("b", "p", lbp)
        net.add_lane("p", "k", lpk)
        net.add_lane("k", "j", lkj)
        road = Road(network=net, np_random=self.env.np_random, record_history=self.record_history)
        # road.objects.append(Obstacle(road, lbp.position(ends[2], 0)))

        self.road = road

    def _road_exit(self):
        # road_length = 1000, exit_humans = 400, exit_length = 100
        exit_position = self.exit_humans + self.exit_controlled
        exit_length =  self.exit_length
        after_exit = self.after_exit
        net = RoadNetwork.straight_road_networkv2(self.lanes_count, start=0,
                                                length=exit_position, nodes_str=("0", "1"))
        net = RoadNetwork.straight_road_networkv2(self.lanes_count+ 1, start=exit_position,
                                                length=exit_length, nodes_str=("1", "2"), net=net)
        net = RoadNetwork.straight_road_networkv2(self.lanes_count, start=exit_position + exit_length,
                                                length=after_exit,
                                                nodes_str=("2", "3"), net=net)
        for _from in net.graph:
            for _to in net.graph[_from]:
                for _id in range(len(net.graph[_from][_to])):
                    net.get_lane((_from, _to, _id)).speed_limit = 26 - 3.4 * _id
        exit_position = np.array([exit_position + exit_length, self.lanes_count * CircularLane.DEFAULT_WIDTH])
        radius = 150
        exit_center = exit_position + np.array([0, radius])
        lane = CircularLane(center=exit_center,
                            radius=radius,
                            start_phase=3 * np.pi / 2,
                            end_phase=2 * np.pi,
                            forbidden=True)
        net.add_lane("2", "exit", lane)

        self.road = Road(network=net,
                         np_random=self.env.np_random)

    def _road_closed(self, end=200, after=1000):
        """Create a road composed of straight adjacent lanes."""

        net = RoadNetwork()
        last_lane = 0
        # Highway lanes

        c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
        y = [last_lane + StraightLane.DEFAULT_WIDTH, last_lane + 2 * StraightLane.DEFAULT_WIDTH]
        line_type = [[c, s], [n, c]]
        line_type_merge = [[c, s], [n, s]]

        new_lane = StraightLane([0, last_lane], [end, last_lane], line_types=[c, n], forbidden=True)
        net.add_lane("a", "b", new_lane)

        for i in range(self.self.lanes_count):
            net.add_lane("a", "b", StraightLane([0, y[i]], [end, y[i]], line_types=line_type[i]))
            net.add_lane("b", "c",
                         StraightLane([end, y[i]], [after, y[i]], line_types=line_type_merge[i]))

        road = Road(network=net, np_random=self.env.np_random, record_history=self.record_history)

        pos = new_lane.position(end, 0)
        road.objects.append(Obstacle(road, pos))
        self.road = road

    def _road_highway(self) -> None:
        """Create a road composed of straight adjacent lanes."""
        self.road = Road(network=RoadNetwork.straight_road_network(self.lanes_count),
                         np_random=self.env.np_random, record_history=self.record_history)
    def _vehicles_highway(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        self.controlled_vehicles = []
        back = False
        basic = False
        if back:
            for _ in range(self.controlled_vehicles_count):
                vehicle = self.env.action_type.vehicle_class.create_random(self.road,
                                                                           speed=25,
                                                                           spacing=self.ego_spacing)
                self.controlled_vehicles.append(vehicle)
                self.road.vehicles.append(vehicle)

            # vehicles_type = cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])

            cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])

            vehicles_type = utils.class_from_path(self.other_vehicles_type)
            for _ in range(self.cruising_vehicles_count):
                self.road.vehicles.append(
                    vehicles_type.create_random(self.road, spacing=1 / self.vehicles_density))
        elif basic:
            other_vehicles_type = utils.class_from_path(self.other_vehicles_type)
            other_per_controlled = near_split(self.cruising_vehicles_count,
                                              num_bins=self.controlled_vehicles_count)

            self.controlled_vehicles = []
            for others in other_per_controlled:
                controlled_vehicle = self.env.action_type.vehicle_class.create_random(
                    self.road,
                    speed=25,
                    spacing=self.ego_spacing
                )
                self.controlled_vehicles.append(controlled_vehicle)
                self.road.vehicles.append(controlled_vehicle)

                for _ in range(others):
                    self.road.vehicles.append(
                        other_vehicles_type.create_random(self.road, spacing=1 / self.vehicles_density)
                    )
        else:
            vehicle_id =1
            cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
            other_per_controlled = near_split(self.cruising_vehicles_count,
                                              num_bins=self.controlled_vehicles_count)

            self.controlled_vehicles = []
            speed_controlled = self.controlled_vehicle_speed
            controlled_vehicle_id = self.cruising_vehicles_count +1
            for others in other_per_controlled:

                controlled_vehicle = self.env.action_type.vehicle_class.create_random(
                    self.road,
                    speed=speed_controlled,
                    spacing=self.ego_spacing, id=controlled_vehicle_id
                )
                controlled_vehicle_id+=1

                for _ in range(others):
                    self.road.vehicles.append(
                        cruising_vehicle.create_random_custom(self.road, spacing=1 / self.vehicles_density,config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)

                    )
                    vehicle_id += 1


                self.controlled_vehicles.append(controlled_vehicle)
                self.road.vehicles.append(controlled_vehicle)

            for v in self.road.vehicles:  # Prevent early collisions
                if v is not controlled_vehicle and np.linalg.norm(v.position - controlled_vehicle.position) < 25:
                    self.road.vehicles.remove(v)


    def _vehicles_road_closed(self, controlled_vehicles=4, cruising_vehicles_count=10) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        self.controlled_vehicles = []

        road = self.road
        vehicle_space = self.converging_merging / controlled_vehicles
        pos = self.before_merging
        for _ in range(controlled_vehicles):
            vehicle = self.env.action_type.vehicle_class(road,
                                                         road.network.get_lane(("a", "b", 1)).position(pos, 0),
                                                         speed=30)
            pos += vehicle_space

            self.controlled_vehicles.append(vehicle)
            road.vehicles.append(vehicle)

        other_vehicles_type = utils.class_from_path(self.other_vehicles_type)

        # spawn vehicles in lane and possition
        road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(80, 0), speed=29))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(30, 0), speed=31))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(5, 0), speed=31.5))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(90, 0), speed=29))

        pos = 0
        vehicle_space = self.before_merging / cruising_vehicles_count
        for i in range(cruising_vehicles_count):
            # spawn vehicles in lane and possition
            road.vehicles.append(
                CustomVehicle(road, road.network.get_lane(("a", "b", 1)).position(pos, 0), config=self.env.config,
                              speed=29,
                              enable_lane_change=False, id=i + 1))
            # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(30, 0), speed=31))
            pos += vehicle_space

        self.road = road

    def _vehicles_merge_to_highway(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        self.controlled_vehicles = []
        road = self.road
        right_lane = len(self.road.network.graph['a']['b']) - 1
        vehicle_id = 1

        vehicle_position = 0
        vehicle_space = self.before_merging / self.cruising_vehicles_count
        if vehicle_space <= (abs(self.random_offset_vehicles[0]) + self.cruising_vehicle['length']):
            exit() #comment
            print(" warning, reduce number of vehicle or offset range")
            # TODO , define default for this case
            # TODO , consider speed also for positioning

        cruising_vehicle_class = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
        speed = self.cruising_vehicle["speed"]
        enable_lane_change = self.cruising_vehicle["enable_lane_change"]

        for i in range(self.cruising_vehicles_count-1):
            if self.randomize_vehicles:
                random_offset = self.random_offset_vehicles
                delta = np.random.randint(low=random_offset[0], high=random_offset[1])

                vehicle_position += delta
                vehicle_position = max(0, vehicle_position)
                # vehicle_position = min(vehicle_position, self.before)
            if self.randomize_speed:
                # speed = road.np_random.uniform(Vehicle.DEFAULT_SPEEDS[0], Vehicle.DEFAULT_SPEEDS[1])
                random_offset = self.randomize_speed_offset
                delta = np.random.randint(low=random_offset[0], high=random_offset[1])
                speed += delta
            vehicle = cruising_vehicle_class(road,
                                             road.network.get_lane(("a", "b", right_lane)).position(vehicle_position,
                                                                                                    0),
                                             speed=speed, enable_lane_change=enable_lane_change,
                                             config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)
            vehicle_position += vehicle_space

            road.vehicles.append(vehicle)
            vehicle_id += 1


    # controlled vehicles
        vehicle_space = self.converging_merging / self.controlled_vehicles_count
        vehicle_position = max(vehicle_position + self.cruising_vehicle['length'], self.before_merging + self.random_offset_vehicles[1])

        baseline_vehicle_class = utils.class_from_path(self.baseline_vehicle["vehicles_type"])
        if self.controlled_baseline_vehicle:
            speed = self.baseline_vehicle["speed"]
        else:
            speed = self.controlled_vehicle_speed
        enable_lane_change = self.baseline_vehicle["enable_lane_change"]

        if vehicle_space <= (abs(self.random_offset_vehicles[0]) + self.cruising_vehicle['length']):
            exit()
            print(" warning, reduce number of vehicle or offset range")
            # TODO , define default for this case
            # TODO , consider speed also for positioning

        # count = 0
        # TODO fix it
        multi_agent_setting = True
        controlled_vehicle_id = self.cruising_vehicles_count + 1

        for _ in range(self.controlled_vehicles_count):
            if self.randomize_vehicles:
                random_offset = self.random_offset_vehicles
                delta = np.random.randint(low=random_offset[0], high=random_offset[1])

                vehicle_position += delta
                vehicle_position = max(0, vehicle_position)

            if self.randomize_speed:
                # speed = road.np_random.uniform(Vehicle.DEFAULT_SPEEDS[0], Vehicle.DEFAULT_SPEEDS[1])
                random_offset = self.randomize_speed_offset
                delta = np.random.randint(low=random_offset[0], high=random_offset[1])
                speed += delta

            if self.controlled_baseline_vehicle:
                vehicle = baseline_vehicle_class(road,
                                                 road.network.get_lane(("a", "b", right_lane)).position(
                                                     vehicle_position,
                                                     0),
                                                 speed=speed, enable_lane_change=enable_lane_change,
                                                 config=self.env.config, v_type='baseline_vehicle', id=vehicle_id)
            else:
                vehicle = self.env.action_type.vehicle_class(road,
                                                             road.network.get_lane(("a", "b", right_lane)).position(
                                                                 vehicle_position, 0),
                                                             speed=speed, id=controlled_vehicle_id)
                # count +=1
                # if count<=2 or multi_agent_setting:
                #     vehicle = self.env.action_type.vehicle_class(road,
                #                                                  road.network.get_lane(("a", "b", right_lane)).position(
                #                                                      vehicle_position, 0),
                #                                                  speed=speed, id=vehicle_id)
                #     if not multi_agent_setting:
                #         vehicle_space = vehicle_space
                # else:
                #     vehicle = cruising_vehicle_class(road,
                #                                      road.network.get_lane(("a", "b", right_lane)).position(
                #                                          vehicle_position,
                #                                          0),
                #                                      speed=speed, enable_lane_change=enable_lane_change,
                #                                      config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)


            vehicle_position += vehicle_space
            self.controlled_vehicles.append(vehicle)
            # if count <= 2 or multi_agent_setting:
            #     self.controlled_vehicles.append(vehicle)



            road.vehicles.append(vehicle)
            controlled_vehicle_id += 1

        if self.cruising_vehicles_front:
            # vehicle_position = max(vehicle_position, self.cruising_vehicles_front_initial_position)
            lane = road.network.get_lane(("b", "c", right_lane))
            last_vehicle_position= lane.local_coordinates(vehicle.position)[0]
            vehicle_position = max(last_vehicle_position + self.ego_spacing * self.cruising_vehicle['length'], self.cruising_vehicles_front_initial_position)
            # vehicle_position = self.cruising_vehicles_front_initial_position
            vehicle_space = self.ego_spacing * self.cruising_vehicle['length']
            enable_lane_change = self.cruising_vehicle["enable_lane_change"]
            speed = self.cruising_vehicle["speed"]
            if vehicle_space <= (abs(self.random_offset_vehicles[0]) + self.cruising_vehicle['length']):
                print(" warning, reduce number of vehicle or offset range")
                exit()
                # TODO , define default for this case
                # TODO , consider speed also for positioning

            for i in range(self.cruising_vehicles_front_count):

                if self.cruising_vehicles_front_random_everywhere:
                    vehicle = self.create_random(cruising_vehicle_class, from_options=["a"],enable_lane_change = self.cruising_vehicle["enable_lane_change"], vehicle_id =vehicle_id)
                else:
                    if self.randomize_vehicles:
                        random_offset = self.random_offset_vehicles
                        delta = np.random.randint(low=random_offset[0], high=random_offset[1])

                        vehicle_position += delta
                        vehicle_position = max(0, vehicle_position)
                        # vehicle_position = min(vehicle_position, self.before)
                    if self.randomize_speed:
                        # speed = road.np_random.uniform(Vehicle.DEFAULT_SPEEDS[0], Vehicle.DEFAULT_SPEEDS[1])
                        random_offset = self.randomize_speed_offset
                        delta = np.random.randint(low=random_offset[0], high=random_offset[1])
                        speed += delta

                    vehicle = cruising_vehicle_class(road,
                                                     road.network.get_lane(("b", "c", right_lane)).position(
                                                         vehicle_position,
                                                         0),
                                                     speed=speed, enable_lane_change=self.cruising_vehicle["enable_lane_change"],
                                                     config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)
                    vehicle_position += vehicle_space

                road.vehicles.append(vehicle)
                vehicle_id += 1

        id_merging_vehicle = self.merging_vehicle['id']
        speed = self.merging_vehicle['speed']
        # TODO check everytime we cahnge a var
        initial_position = self.merging_vehicle['initial_position']

        if self.complex:
            self.merging_vehicle['randomize'] = True
            # self.merging_vehicle['random_offset_merging'] = [-20,20]
            self.merging_vehicle['random_offset_merging'] = [-100,50]
            self.merging_vehicle['randomize_speed_merging'] = True
            self.randomize_speed_offset =[-5,5]

        if self.merging_vehicle['randomize']:
            random_offset = self.merging_vehicle['random_offset_merging']
            delta = np.random.randint(low=random_offset[0], high=random_offset[1])
            # delta = np.random.normal(0, random_offset[1] / 3)
            if delta > 0:
                delta = min(delta, random_offset[1])
            else:
                delta = max(delta, random_offset[0])

            initial_position[0] += delta
            initial_position[0] = max(0, initial_position[0])



        if self.merging_vehicle['randomize_speed_merging']:
            # speed = road.np_random.uniform(Vehicle.DEFAULT_SPEEDS[0], Vehicle.DEFAULT_SPEEDS[1])
            random_offset = self.randomize_speed_offset
            delta = np.random.randint(low=random_offset[0], high=random_offset[1])
            # delta = np.random.normal(0, random_offset[1]/3)
            if delta > 0:
                delta = min(delta,random_offset[1])
            else:
                delta = max(delta, random_offset[0])

            speed += delta
            speed = max(0, speed)

        route = None
        if self.merging_vehicle['controlled_vehicle']:
            # if  self.exit_vehicle['set_route']:
            #     route=[('j', 'k', 0), ('k', 'b', 0), ('b', 'c', 0),('c', 'd', 0)]

            merging_v = self.env.action_type.vehicle_class(road,
                                                           road.network.get_lane(("j", "k", 0)).position(
                                                               initial_position[0],
                                                               initial_position[1]), speed=speed,
                                                           config=self.env.config, id=id_merging_vehicle, route=route , min_speed=self.merging_vehicle['min_speed'], max_speed=self.merging_vehicle['max_speed'])
        else:
            # route = [('j', 'k', 0), ('k', 'b', 0), ('b', 'c', 0), ('c', 'd', 0)]
            merging_vehicle = utils.class_from_path(self.merging_vehicle['vehicles_type'])

            merging_v = merging_vehicle(road, road.network.get_lane(("j", "k", 0)).position(initial_position[0],
                                                                                            initial_position[1]),
                                        speed=speed,
                                        config=self.env.config, v_type='merging_vehicle', id=id_merging_vehicle,
                                        route=route)

        road.vehicles.append(merging_v)
        if self.merging_vehicle['controlled_vehicle']:
            self.controlled_vehicles.append(merging_v)
        self.road = road

    def _vehicles_exit_highway1(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        # TODO always in the same possition ??
        # random.seed(30)

        self.controlled_vehicles = []

        road = self.road

        right_lane = len(self.road.network.graph['a']['b']) - 1
        vehicle_space = self.converging_exit / self.controlled_vehicles_count
        vehicle_position = self.before_exit
        vehicle_id = 1
        if self.randomize_vehicles:
            random_offset = self.random_offset_vehicles
            vehicle_position += random_offset[1]

        baseline_vehicle_class = utils.class_from_path(self.baseline_vehicle["vehicles_type"])
        speed = self.baseline_vehicle["speed"]
        enable_lane_change = self.baseline_vehicle["enable_lane_change"]

        for _ in range(self.controlled_vehicles_count):
            if self.controlled_baseline_vehicle:
                vehicle = baseline_vehicle_class(road,
                                                 road.network.get_lane(("a", "b", right_lane)).position(
                                                     vehicle_position,
                                                     0),
                                                 speed=speed, enable_lane_change=enable_lane_change,
                                                 config=self.env.config, v_type='baseline_vehicle', id=vehicle_id)
                vehicle.is_controlled = 1

            else:
                vehicle = self.env.action_type.vehicle_class(road,
                                                             road.network.get_lane(("a", "b", right_lane)).position(
                                                                 vehicle_position, 0),
                                                             speed=30, id=vehicle_id)
            vehicle_position += vehicle_space

            self.controlled_vehicles.append(vehicle)
            road.vehicles.append(vehicle)
            vehicle_id += 1

        vehicle_position = 0
        vehicle_space = self.before_exit / self.cruising_vehicles_count

        cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
        # TODO ? speed random?
        speed = self.cruising_vehicle['speed']
        enable_lane_change = self.cruising_vehicle['enable_lane_change']

        for i in range(self.cruising_vehicles_count):
            # spawn vehicles in lane and possition
            # if self.env.config['scenario']['randomize_vehicles']:
            #     # vehicle = cruising_vehicle.create_random(road,spacing=self.env.config["ego_spacing"],id=vehicle_id)
            #     vehicle_position
            # else:
            #     vehicle = cruising_vehicle(road, road.network.get_lane(("a", "b", right_lane)).position(vehicle_position, 0), speed=speed,enable_lane_change=enable_lane_change,
            #                       config=self.env.config,v_type='cruising_vehicle',id=vehicle_id)
            #     vehicle_position += vehicle_space

            if self.randomize_vehicles:
                random_offset = self.random_offset_vehicles
                delta = np.random.randint(low=random_offset[0], high=random_offset[1])

                vehicle_position += delta
                vehicle_position = max(0, vehicle_position)
                # vehicle_position = min(vehicle_position, self.before)
            vehicle = cruising_vehicle(road,
                                       road.network.get_lane(("a", "b", right_lane)).position(vehicle_position, 0),
                                       speed=speed, enable_lane_change=enable_lane_change,
                                       config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)
            vehicle_position += vehicle_space

            road.vehicles.append(vehicle)
            vehicle_id += 1

        id_exit_vehicle = self.exit_vehicle['id']
        speed = self.exit_vehicle['speed']
        initial_position = self.exit_vehicle['initial_position']

        if self.exit_vehicle['randomize']:
            random_offset = self.random_offset_vehicles
            delta = np.random.randint(low=random_offset[0], high=random_offset[1])
            initial_position[0] += delta
            initial_position[0] = max(0, initial_position[0])

        route = None
        if self.exit_vehicle['controlled_vehicle']:
            if self.exit_vehicle['set_route']:
                route = [('a', 'b', 0), ('b', 'p', 0), ('p', 'k', 0), ('k', 'j', 0)]

            exit_v = self.env.action_type.vehicle_class(road,
                                                        road.network.get_lane(("a", "b", 0)).position(
                                                            initial_position[0],
                                                            initial_position[1]), speed=speed, config=self.env.config,
                                                        id=id_exit_vehicle, route=route)
        else:
            exit_vehicle = utils.class_from_path(self.exit_vehicle["vehicles_type"])
            route = [('a', 'b', 0), ('b', 'p', 0), ('p', 'k', 0), ('k', 'j', 0)]
            exit_v = exit_vehicle(road, road.network.get_lane(("a", "b", 0)).position(initial_position[0],
                                                                                      initial_position[1]),
                                  speed=speed,
                                  config=self.env.config, v_type='exit_vehicle', id=id_exit_vehicle,
                                  route=route)

        road.vehicles.append(exit_v)
        if self.merging_vehicle['controlled_vehicle']:
            self.controlled_vehicles.append(exit_v)
        self.road = road

    def _vehicles_exit_highwayv0(self):
        """Create some new random vehicles of a given type, and add them on the road."""
        self.controlled_vehicles = []
        road = self.road
        for _ in range(self.controlled_vehicles_count):
            vehicle = self.env.action_type.vehicle_class.create_randomv2(road,
                                                                         speed=25,
                                                                         lane_from="0",
                                                                         lane_to="1",
                                                                         lane_id=0,
                                                                         spacing=self.ego_spacing)
            vehicle.SPEED_MIN = 18
            self.controlled_vehicles.append(vehicle)
            road.vehicles.append(vehicle)

        # vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        cruising_vehicle_class = vehicles_type = utils.class_from_path(self.cruising_vehicle["vehicles_type"])

        for _ in range(self.cruising_vehicles_count):
            lanes = np.arange(self.lanes_count)
            lane_id = road.np_random.choice(lanes, size=1,
                                            p=lanes / lanes.sum()).astype(int)[0]
            lane = road.network.get_lane(("0", "1", lane_id))
            vehicle = vehicles_type.create_randomv2(road,
                                                    lane_from="0",
                                                    lane_to="1",
                                                    lane_id=lane_id,
                                                    speed=lane.speed_limit,
                                                    spacing=1 / self.vehicles_density,
                                                    ).plan_route_to("3")
            vehicle.enable_lane_change = False
            road.vehicles.append(vehicle)

        self.road = road
    def _vehicles_exit_highway(self):
        """Create some new random vehicles of a given type, and add them on the road."""
        self.controlled_vehicles = []
        road = self.road
        right_lane = 1
        vehicle_position = self.exit_humans + 10
        vehicle_space = self.exit_controlled/ self.controlled_vehicles_count
        vehicle_id = 1
        controlled_vehicle_id = self.cruising_vehicles_count + 1

        speed = self.controlled_vehicle_speed
        for _ in range(self.controlled_vehicles_count):
            vehicle = self.env.action_type.vehicle_class(road,
                                                             road.network.get_lane(("0", "1", right_lane)).position(
                                                                 vehicle_position, 0),
                                                             speed=speed, id=controlled_vehicle_id)
            vehicle_position += vehicle_space

            controlled_vehicle_id += 1
            self.controlled_vehicles.append(vehicle)
            road.vehicles.append(vehicle)

        # vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        cruising_vehicle_class= vehicles_type = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
        vehicle_position = 0
        vehicle_space = (self.exit_humans) / self.cruising_vehicles_count
        speed = self.cruising_vehicle['speed']
        enable_lane_change = self.cruising_vehicle['enable_lane_change']
        cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
        for _ in range(self.cruising_vehicles_count-1):
            vehicle = cruising_vehicle(road,
                                       road.network.get_lane(("0", "1", 1)).position(vehicle_position, 0),
                                       speed=speed, enable_lane_change=enable_lane_change,
                                       config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)
            vehicle_position += vehicle_space
            road.vehicles.append(vehicle)
            vehicle_id += 1

        id_exit_vehicle = self.exit_vehicle['id']
        speed = self.exit_vehicle['speed']
        initial_position = self.exit_vehicle['initial_position']
        route = None
        if self.complex:
            self.exit_vehicle['randomize'] = True
            self.exit_vehicle['random_offset_exit'] =[-100,50]
            # self.exit_vehicle['random_offset_exit'] =[-30,30]

        if self.exit_vehicle['randomize']:
            episode = self.env.episode
            # initial_positions=[69,74]
            # idx = episode % 2

            # if episode%150 ==0:
            #     idx = 1
            # else:
            #     idx =0
            # initial_position[0] = initial_positions[idx]

            random_offset = self.exit_vehicle['random_offset_exit']
            delta = np.random.randint(low=random_offset[0], high=random_offset[1])
            # delta = np.random.normal(0, random_offset[1] / 3)
            # if delta > 0:
            #     delta = min(delta, random_offset[1])
            # else:
            #     delta = max(delta, -random_offset[1])

            initial_position[0] += delta
            # initial_position[0] = max(0, initial_position[0])

        if self.exit_vehicle['controlled_vehicle']:
            if self.exit_vehicle['set_route']:
                route = [('a', 'b', 0), ('b', 'p', 0), ('p', 'k', 0), ('k', 'j', 0)]

            exit_v = self.env.action_type.vehicle_class(road,
                                                        road.network.get_lane(("a", "b", 0)).position(
                                                            initial_position[0],
                                                            initial_position[1]), speed=speed, config=self.env.config,
                                                        id=id_exit_vehicle, route=route)
        else:
            exit_vehicle = utils.class_from_path(self.exit_vehicle["vehicles_type"])
            route = [('0', '1', 1), ('1', '2', 2), ('2', 'exit', 0)]
            exit_v = exit_vehicle(road, road.network.get_lane(("0", "1", 0)).position(initial_position[0],
                                                                                      initial_position[1]),
                                  speed=speed,
                                  config=self.env.config, v_type='exit_vehicle', id=id_exit_vehicle,
                                  route=route)

        road.vehicles.append(exit_v)
        if self.merging_vehicle['controlled_vehicle']:
            self.controlled_vehicles.append(exit_v)


        self.road = road

    def _vehicles_merge_to_highway_prob(self) -> None:
        """Create some new random vehicles of a given type, and add them on the road."""
        self.controlled_vehicles = []

        road = self.road

        right_lane = len(self.road.network.graph['a']['b']) - 1
        # total_vehicle_count=self.controlled_vehicles_count + self.cruising_vehicles
        total_vehicle_count = self.total_number_of_vehicles
        vehicle_space = (self.converging_merging + self.before_merging) / (total_vehicle_count)
        vehicle_position = 0
        vehicle_id = 1
        prob_of_controlled_vehicle = self.prob_of_controlled_vehicle

        cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
        speed = self.cruising_vehicle['speed']
        enable_lane_change = self.cruising_vehicle['enable_lane_change']

        # TODO always in the same possition ??
        random.seed(30)

        for _ in range(total_vehicle_count):
            rand_check = random.random()  # random.uniform(a, b) ,  random.gauss(mu, sigma)
            if self.randomize_vehicles:
                random_offset = self.random_offset_vehicles
                delta = np.random.randint(low=random_offset[0], high=random_offset[1])

                vehicle_position += delta
                vehicle_position = max(0, vehicle_position)

            if rand_check <= prob_of_controlled_vehicle:
                vehicle = self.env.action_type.vehicle_class(road,
                                                             road.network.get_lane(("a", "b", right_lane)).position(
                                                                 vehicle_position, 0),
                                                             speed=30, id=vehicle_id)
                self.controlled_vehicles.append(vehicle)
            else:
                vehicle = cruising_vehicle(road,
                                           road.network.get_lane(("a", "b", right_lane)).position(vehicle_position, 0),
                                           speed=speed, enable_lane_change=enable_lane_change,
                                           config=self.env.config, v_type='cruising_vehicle', id=vehicle_id)
            vehicle_position += vehicle_space

            road.vehicles.append(vehicle)
            vehicle_id += 1

        id_merging_vehicle = self.merging_vehicle['id']
        speed = self.merging_vehicle['speed']
        initial_position = self.merging_vehicle['initial_position']

        if self.merging_vehicle['controlled_vehicle']:
            merging_v = self.env.action_type.vehicle_class(road,
                                                           road.network.get_lane(("j", "k", 0)).position(
                                                               initial_position[0],
                                                               initial_position[1]), speed=speed, id=id_merging_vehicle)
        else:
            merging_vehicle = utils.class_from_path(self.merging_vehicle["vehicles_type"])

            merging_v = merging_vehicle(road, road.network.get_lane(("j", "k", 0)).position(initial_position[0],
                                                                                            initial_position[1]),
                                        speed=speed,
                                        config=self.env.config, v_type='merging_vehicle', id=id_merging_vehicle)

        road.vehicles.append(merging_v)
        if self.merging_vehicle['controlled_vehicle']:
            self.controlled_vehicles.append(merging_v)
        self.road = road

    def _road_intersection(self):
        """
        Make an 4-way intersection.

        The horizontal road has the right of way. More precisely, the levels of priority are:
            - 3 for horizontal straight lanes and right-turns
            - 1 for vertical straight lanes and right-turns
            - 2 for horizontal left-turns
            - 0 for vertical left-turns

        The code for nodes in the road network is:
        (o:outer | i:inner + [r:right, l:left]) + (0:south | 1:west | 2:north | 3:east)

        :return: the intersection road
        """
        lane_width = AbstractLane.DEFAULT_WIDTH
        right_turn_radius = lane_width + 5  # [m}
        left_turn_radius = right_turn_radius + lane_width  # [m}
        outer_distance = right_turn_radius + lane_width / 2
        access_length = 50 + 120  # [m]

        net = RoadNetwork()
        n, c, s = LineType.NONE, LineType.CONTINUOUS, LineType.STRIPED
        for corner in range(4):
            angle = np.radians(90 * corner)
            is_horizontal = corner % 2
            priority = 3 if is_horizontal else 1
            rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            # Incoming
            start = rotation @ np.array([lane_width / 2, access_length + outer_distance])
            end = rotation @ np.array([lane_width / 2, outer_distance])
            net.add_lane("o" + str(corner), "ir" + str(corner),
                         StraightLane(start, end, line_types=[s, c], priority=priority, speed_limit=10))
            # Right turn
            r_center = rotation @ (np.array([outer_distance, outer_distance]))
            net.add_lane("ir" + str(corner), "il" + str((corner - 1) % 4),
                         CircularLane(r_center, right_turn_radius, angle + np.radians(180), angle + np.radians(270),
                                      line_types=[n, c], priority=priority, speed_limit=10))
            # Left turn
            l_center = rotation @ (np.array([-left_turn_radius + lane_width / 2, left_turn_radius - lane_width / 2]))
            net.add_lane("ir" + str(corner), "il" + str((corner + 1) % 4),
                         CircularLane(l_center, left_turn_radius, angle + np.radians(0), angle + np.radians(-90),
                                      clockwise=False, line_types=[n, n], priority=priority - 1, speed_limit=10))
            # Straight
            start = rotation @ np.array([lane_width / 2, outer_distance])
            end = rotation @ np.array([lane_width / 2, -outer_distance])
            net.add_lane("ir" + str(corner), "il" + str((corner + 2) % 4),
                         StraightLane(start, end, line_types=[s, n], priority=priority, speed_limit=10))
            # Exit
            start = rotation @ np.flip([lane_width / 2, access_length + outer_distance], axis=0)
            end = rotation @ np.flip([lane_width / 2, outer_distance], axis=0)
            net.add_lane("il" + str((corner - 1) % 4), "o" + str((corner - 1) % 4),
                         StraightLane(end, start, line_types=[n, c], priority=priority, speed_limit=10))

        road = RegulatedRoad(network=net, np_random=self.env.np_random, record_history=self.env.config["show_trajectories"])
        self.road = road
    def _vehicles_intersection(self):
        """
                Populate a road with several vehicles on the highway and on the merging lane

                :return: the ego-vehicle
                """
        # Configure vehicles
        baseline_vehicle = utils.class_from_path(self.baseline_vehicle["vehicles_type"])

        vehicle_type = baseline_vehicle
        vehicle_type.DISTANCE_WANTED = 7  # Low jam distance
        vehicle_type.COMFORT_ACC_MAX = 6
        vehicle_type.COMFORT_ACC_MIN = -3

        controlled_vehicle_id = self.cruising_vehicles_count + 1
        vehicle_id = 1
        # Random vehicles
        simulation_steps = 3
        # vehicle_count = self.cruising_vehicles_count
        if self.complex:
            self.cruising_vehicles_count = self.cruising_vehicles_count + 3
        if self.simple:
            self.cruising_vehicles_count =6
            self.env.config["scenario"]["intersection"]["spawn_probability"] = 0.4
        for t in range(self.cruising_vehicles_count - 1):
            self._spawn_vehicle_intersection(np.linspace(0, 140, self.cruising_vehicles_count)[t],vehicle_id=vehicle_id,spawn_probability=self.scenario_config["intersection"]["spawn_probability"])
            vehicle_id += 1
        for _ in range(simulation_steps):
            [(self.road.act(), self.road.step(1 / self.simulation_frequency )) for _ in
             range(self.simulation_frequency )]

        # Challenger vehicle
        self._spawn_vehicle_intersection(60, spawn_probability=1, go_straight=True, position_deviation=0.1, speed_deviation=0,vehicle_id=vehicle_id)
        vehicle_id += 1
        # Controlled vehicles
        self.controlled_vehicles = []
        for ego_id in range(0, self.controlled_vehicles_count):
            ego_lane = self.road.network.get_lane(("o{}".format(ego_id % 4), "ir{}".format(ego_id % 4), 0))
            destination = self.scenario_config["intersection"]["destination"] or "o" + str(self.np_random.randint(1, 4))
            ego_vehicle = self.env.action_type.vehicle_class(
                self.road,
                ego_lane.position(60 + 5 * self.env.np_random.randn(1), 0),
                speed=ego_lane.speed_limit,
                heading=ego_lane.heading_at(60), id=controlled_vehicle_id,config=self.env.config)

            controlled_vehicle_id += 1

            try:
                ego_vehicle.plan_route_to(destination)
                # self.env.config['controlled_vehicle']['min_speed'] = 5
                # self.env.config['controlled_vehicle']['max_speed'] = 20
                ego_vehicle.SPEED_MIN = 5
                ego_vehicle.SPEED_MAX = 20
                ego_vehicle.SPEED_COUNT = 3
                ego_vehicle.speed_index = ego_vehicle.speed_to_index(ego_lane.speed_limit)
                ego_vehicle.target_speed = ego_vehicle.index_to_speed(ego_vehicle.speed_index)
            except AttributeError:
                pass

            self.road.vehicles.append(ego_vehicle)
            self.controlled_vehicles.append(ego_vehicle)
            for v in self.road.vehicles:  # Prevent early collisions
                if v is not ego_vehicle and np.linalg.norm(v.position - ego_vehicle.position) < 25:
                    self.road.vehicles.remove(v)

    def _spawn_vehicle_intersection(self,
                       longitudinal: float = 0,
                       position_deviation: float = 1.,
                       speed_deviation: float = 1.,
                       spawn_probability: float = 0.6,
                       go_straight: bool = False,vehicle_id=0) -> None:
        if self.env.np_random.rand() > spawn_probability:
            return

        route = self.env.np_random.choice(range(4), size=2, replace=False)
        route[1] = (route[0] + 2) % 4 if go_straight else route[1]

        baseline_vehicle = utils.class_from_path(self.baseline_vehicle["vehicles_type"])

        vehicle_type = baseline_vehicle
        vehicle = vehicle_type.make_on_lane(self.road, ("o" + str(route[0]), "ir" + str(route[0]), 0),
                                            longitudinal=longitudinal + 5 + self.env.np_random.randn() * position_deviation,
                                            speed=8 + self.env.np_random.randn() * speed_deviation,id=vehicle_id)
        for v in self.road.vehicles:
            if np.linalg.norm(v.position - vehicle.position) < 25:
                return
        vehicle.plan_route_to("o" + str(route[1]))
        # vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)
        return vehicle

    def _road_roundabout(self):
        # Circle lanes: (s)outh/(e)ast/(n)orth/(w)est (e)ntry/e(x)it.
        center = [0, 0]  # [m]
        radius = 20  # [m]
        alpha = 24  # [deg]

        net = RoadNetwork()
        radii = [radius, radius + 4]
        n, c, s = LineType.NONE, LineType.CONTINUOUS, LineType.STRIPED
        line = [[c, s], [n, c]]
        for lane in [0, 1]:
            net.add_lane("se", "ex",
                         CircularLane(center, radii[lane], np.deg2rad(90 - alpha), np.deg2rad(alpha),
                                      clockwise=False, line_types=line[lane]))
            net.add_lane("ex", "ee",
                         CircularLane(center, radii[lane], np.deg2rad(alpha), np.deg2rad(-alpha),
                                      clockwise=False, line_types=line[lane]))
            net.add_lane("ee", "nx",
                         CircularLane(center, radii[lane], np.deg2rad(-alpha), np.deg2rad(-90 + alpha),
                                      clockwise=False, line_types=line[lane]))
            net.add_lane("nx", "ne",
                         CircularLane(center, radii[lane], np.deg2rad(-90 + alpha), np.deg2rad(-90 - alpha),
                                      clockwise=False, line_types=line[lane]))
            net.add_lane("ne", "wx",
                         CircularLane(center, radii[lane], np.deg2rad(-90 - alpha), np.deg2rad(-180 + alpha),
                                      clockwise=False, line_types=line[lane]))
            net.add_lane("wx", "we",
                         CircularLane(center, radii[lane], np.deg2rad(-180 + alpha), np.deg2rad(-180 - alpha),
                                      clockwise=False, line_types=line[lane]))
            net.add_lane("we", "sx",
                         CircularLane(center, radii[lane], np.deg2rad(180 - alpha), np.deg2rad(90 + alpha),
                                      clockwise=False, line_types=line[lane]))
            net.add_lane("sx", "se",
                         CircularLane(center, radii[lane], np.deg2rad(90 + alpha), np.deg2rad(90 - alpha),
                                      clockwise=False, line_types=line[lane]))

        # Access lanes: (r)oad/(s)ine
        access = 200  # [m]
        dev = 85  # [m]
        a = 5  # [m]
        delta_st = 0.2 * dev  # [m]

        delta_en = dev - delta_st
        w = 2 * np.pi / dev
        net.add_lane("ser", "ses", StraightLane([2, access], [2, dev / 2], line_types=(s, c)))
        net.add_lane("ses", "se",
                     SineLane([2 + a, dev / 2], [2 + a, dev / 2 - delta_st], a, w, -np.pi / 2, line_types=(c, c)))
        net.add_lane("sx", "sxs",
                     SineLane([-2 - a, -dev / 2 + delta_en], [-2 - a, dev / 2], a, w, -np.pi / 2 + w * delta_en,
                              line_types=(c, c)))
        net.add_lane("sxs", "sxr", StraightLane([-2, dev / 2], [-2, access], line_types=(n, c)))

        net.add_lane("eer", "ees", StraightLane([access, -2], [dev / 2, -2], line_types=(s, c)))
        net.add_lane("ees", "ee",
                     SineLane([dev / 2, -2 - a], [dev / 2 - delta_st, -2 - a], a, w, -np.pi / 2, line_types=(c, c)))
        net.add_lane("ex", "exs",
                     SineLane([-dev / 2 + delta_en, 2 + a], [dev / 2, 2 + a], a, w, -np.pi / 2 + w * delta_en,
                              line_types=(c, c)))
        net.add_lane("exs", "exr", StraightLane([dev / 2, 2], [access, 2], line_types=(n, c)))

        net.add_lane("ner", "nes", StraightLane([-2, -access], [-2, -dev / 2], line_types=(s, c)))
        net.add_lane("nes", "ne",
                     SineLane([-2 - a, -dev / 2], [-2 - a, -dev / 2 + delta_st], a, w, -np.pi / 2, line_types=(c, c)))
        net.add_lane("nx", "nxs",
                     SineLane([2 + a, dev / 2 - delta_en], [2 + a, -dev / 2], a, w, -np.pi / 2 + w * delta_en,
                              line_types=(c, c)))
        net.add_lane("nxs", "nxr", StraightLane([2, -dev / 2], [2, -access], line_types=(n, c)))

        net.add_lane("wer", "wes", StraightLane([-access, 2], [-dev / 2, 2], line_types=(s, c)))
        net.add_lane("wes", "we",
                     SineLane([-dev / 2, 2 + a], [-dev / 2 + delta_st, 2 + a], a, w, -np.pi / 2, line_types=(c, c)))
        net.add_lane("wx", "wxs",
                     SineLane([dev / 2 - delta_en, -2 - a], [-dev / 2, -2 - a], a, w, -np.pi / 2 + w * delta_en,
                              line_types=(c, c)))
        net.add_lane("wxs", "wxr", StraightLane([-dev / 2, -2], [-access, -2], line_types=(n, c)))

        road = Road(network=net, np_random=self.np_random, record_history=self.env.config["show_trajectories"])
        self.road = road
    def _vehicles_roundabout(self):
        """
                Populate a road with several vehicles on the highway and on the merging lane, as well as an ego-vehicle.

                :return: the ego-vehicle
                """
        position_deviation = 2
        speed_deviation = 2

        self.controlled_vehicles = []
        # Ego-vehicle
        controlled_vehicle_id = self.cruising_vehicles_count + 1

        ego_lane = self.road.network.get_lane(("ser", "ses", 0))
        ego_vehicle = self.env.action_type.vehicle_class(self.road,
                                                     ego_lane.position(125, 0),
                                                     speed=8,
                                                     heading=ego_lane.heading_at(140), id=controlled_vehicle_id,config=self.env.config)
        try:
            ego_vehicle.plan_route_to("nxs")
        except AttributeError:
            pass
        # self.env.config['controlled_vehicle']['min_speed'] = 5
        # self.env.config['controlled_vehicle']['max_speed'] = 20
        ego_vehicle.SPEED_MIN = 5
        ego_vehicle.SPEED_MAX = 20
        ego_vehicle.SPEED_COUNT = 3
        self.road.vehicles.append(ego_vehicle)
        self.controlled_vehicles.append(ego_vehicle)
        self.vehicle = ego_vehicle

        vehicle_id =1
        # Incoming vehicle
        destinations = ["exr", "sxr", "nxr"]
        baseline_vehicle = utils.class_from_path(self.baseline_vehicle["vehicles_type"])
        vehicle_type = baseline_vehicle
        vehicle = vehicle_type.make_on_lane(self.road,
                                                   ("we", "sx", 1),
                                                   longitudinal=5 + self.np_random.randn() * position_deviation,
                                                   speed=16 + self.np_random.randn() * speed_deviation,id=vehicle_id)
        vehicle_id+=1
        if self.env.config["scenario"]["roundabout"]["incoming_vehicle_destination"] is not None:
            destination = destinations[self.env.config["scenario"]["roundabout"]["incoming_vehicle_destination"]]
        else:
            destination = self.np_random.choice(destinations)
        vehicle.plan_route_to(destination)
        # vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)

        # Other vehicles
        n = 4
        if self.complex:
            n=5
        if self.simple:
            n = 2
        for i in list(range(1, n)) + list(range(-n+1, 0)):
            vehicle = vehicle_type.make_on_lane(self.road,
                                                ("we", "sx", 0),
                                                longitudinal=20 * i + self.np_random.randn() * position_deviation,
                                                speed=16 + self.np_random.randn() * speed_deviation, id=vehicle_id)
            vehicle.plan_route_to(self.np_random.choice(destinations))
            # vehicle.randomize_behavior()
            safe = True
            for v in self.road.vehicles:  # Prevent early collisions
                if np.linalg.norm(v.position - vehicle.position) < 10:
                    safe = False
                    break
            if safe:
                self.road.vehicles.append(vehicle)
                vehicle_id += 1
        # Entering vehicle
        vehicle = vehicle_type.make_on_lane(self.road,
                                                   ("eer", "ees", 0),
                                                   longitudinal=50 + self.np_random.randn() * position_deviation,
                                                   speed=16 + self.np_random.randn() * speed_deviation,id=vehicle_id)
        vehicle.plan_route_to(self.np_random.choice(destinations))
        vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)

        for v in self.road.vehicles:  # Prevent early collisions
                if v is not ego_vehicle and np.linalg.norm(v.position - ego_vehicle.position) < 15:
                    self.road.vehicles.remove(v)

    def _road_twoway(self):
        """
               Make a road composed of a two-way road.

               :return: the road
               """
        net = RoadNetwork()
        length = 1200
        # Lanes
        net.add_lane("a", "b", StraightLane([0, 0], [length, 0],
                                            line_types=(LineType.CONTINUOUS_LINE, LineType.STRIPED)))
        net.add_lane("a", "b", StraightLane([0, StraightLane.DEFAULT_WIDTH], [length, StraightLane.DEFAULT_WIDTH],
                                            line_types=(LineType.NONE, LineType.CONTINUOUS_LINE)))
        net.add_lane("b", "a", StraightLane([length, 0], [0, 0],
                                            line_types=(LineType.NONE, LineType.NONE)))

        road = Road(network=net, np_random=self.np_random, record_history=self.env.config["show_trajectories"])
        self.road = road
    def _vehicles_twoway(self):
        """
                Populate a road with several vehicles on the road

                :return: the ego-vehicle
                """
        self.controlled_vehicles = []
        controlled_vehicle_id = self.cruising_vehicles_count + 1
        road = self.road
        ego_vehicle = self.env.action_type.vehicle_class(road,
                                                     road.network.get_lane(("a", "b", 1)).position(30, 0),
                                                     speed=self.controlled_vehicle_speed, id=controlled_vehicle_id)
        road.vehicles.append(ego_vehicle)
        self.vehicle = ego_vehicle
        self.controlled_vehicles.append(ego_vehicle)

        cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
        vehicles_type = cruising_vehicle
        vehicle_id = 1
        oneway = int(self.cruising_vehicles_count/2)
        for i in range(oneway):
            self.road.vehicles.append(
                vehicles_type(road,
                              position=road.network.get_lane(("a", "b", 1))
                              .position(70 + 40 * i + 10 * self.np_random.randn(), 0),
                              heading=road.network.get_lane(("a", "b", 1)).heading_at(70 + 40 * i),
                              speed=24 + 2 * self.np_random.randn(),
                              enable_lane_change=False, id=vehicle_id)
            )
            vehicle_id += 1
        for i in range(self.cruising_vehicles_count - oneway):
            v = vehicles_type(road,
                              position=road.network.get_lane(("b", "a", 0))
                              .position(200 + 100 * i + 10 * self.np_random.randn(), 0),
                              heading=road.network.get_lane(("b", "a", 0)).heading_at(200 + 100 * i),
                              speed=20 + 5 * self.np_random.randn(),
                              enable_lane_change=False, id=vehicle_id)
            v.target_lane_index = ("b", "a", 0)
            self.road.vehicles.append(v)
            vehicle_id += 1

        for v in self.road.vehicles:  # Prevent early collisions
            if v is not ego_vehicle and np.linalg.norm(v.position - ego_vehicle.position) < 15:
                self.road.vehicles.remove(v)

    def _road_uturn(self):
        """
               Making double lane road with counter-clockwise U-Turn.
               :return: the road
               """
        net = RoadNetwork()
        length = 128
        # Defining upper starting lanes after the U-Turn.
        # These Lanes are defined from x-coordinate 'length' to 0.
        net.add_lane("c", "d", StraightLane([length, StraightLane.DEFAULT_WIDTH], [0, StraightLane.DEFAULT_WIDTH],
                                            line_types=(LineType.CONTINUOUS_LINE, LineType.STRIPED)))
        net.add_lane("c", "d", StraightLane([length, 0], [0, 0],
                                            line_types=(LineType.NONE, LineType.CONTINUOUS_LINE)))

        # Defining counter-clockwise circular U-Turn lanes.
        center = [length, StraightLane.DEFAULT_WIDTH + 20]  # [m]
        radius = 20  # [m]
        alpha = 0  # [deg]

        radii = [radius, radius + StraightLane.DEFAULT_WIDTH]
        n, c, s = LineType.NONE, LineType.CONTINUOUS, LineType.STRIPED
        line = [[c, s], [n, c]]
        for lane in [0, 1]:
            net.add_lane("b", "c",
                         CircularLane(center, radii[lane], np.deg2rad(90 - alpha), np.deg2rad(-90 + alpha),
                                      clockwise=False, line_types=line[lane]))

        offset = 2 * radius

        # Defining lower starting lanes before the U-Turn.
        # These Lanes are defined from x-coordinate 0 to 'length'.
        net.add_lane("a", "b",
                     StraightLane([0, ((2 * StraightLane.DEFAULT_WIDTH + offset) - StraightLane.DEFAULT_WIDTH)],
                                  [length, ((2 * StraightLane.DEFAULT_WIDTH + offset) - StraightLane.DEFAULT_WIDTH)],
                                  line_types=(LineType.CONTINUOUS_LINE,
                                              LineType.STRIPED)))
        net.add_lane("a", "b", StraightLane([0, (2 * StraightLane.DEFAULT_WIDTH + offset)],
                                            [length, (2 * StraightLane.DEFAULT_WIDTH + offset)],
                                            line_types=(LineType.NONE,
                                                        LineType.CONTINUOUS_LINE)))

        road = Road(network=net, np_random=self.np_random, record_history=self.env.config["show_trajectories"])
        self.road = road
    def _vehicles_uturn(self):
        """
                Strategic addition of vehicles for testing safety behavior limits
                while performing U-Turn manoeuvre at given cruising interval.

                :return: the ego-vehicle
                """

        # These variables add small variations to the driving behavior.
        position_deviation = 2
        speed_deviation = 2
        self.controlled_vehicles = []
        controlled_vehicle_id = self.cruising_vehicles_count + 1

        ego_lane = self.road.network.get_lane(("a", "b", 0))
        ego_vehicle = self.env.action_type.vehicle_class(self.road,
                                                     ego_lane.position(0, 0),
                                                     speed=self.controlled_vehicle_speed, id=controlled_vehicle_id)
        # Stronger anticipation for the turn
        ego_vehicle.PURSUIT_TAU = ego_vehicle.TAU_HEADING
        # Lower speed range
        ego_vehicle.SPEED_MIN = 8
        ego_vehicle.SPEED_MAX = 24
        ego_vehicle.SPEED_COUNT = 3
        try:
            ego_vehicle.plan_route_to("d")
        except AttributeError:
            pass

        self.controlled_vehicles.append(ego_vehicle)
        self.road.vehicles.append(ego_vehicle)
        self.vehicle = ego_vehicle

        cruising_vehicle = utils.class_from_path(self.cruising_vehicle["vehicles_type"])
        vehicles_type = cruising_vehicle
        # Note: randomize_behavior() can be commented out if more randomized
        # vehicle interactions are deemed necessary for the experimentation.

        vehicle_id = 1
        # Vehicle 1: Blocking the ego vehicle
        vehicle = vehicles_type.make_on_lane(self.road,
                                             ("a", "b", 0),
                                             longitudinal=25 + self.np_random.randn() * position_deviation,
                                             speed=13.5 + self.np_random.randn() * speed_deviation, id=vehicle_id)
        vehicle.plan_route_to('d')
        vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)
        vehicle_id += 1

        # Vehicle 2: Forcing risky overtake
        vehicle = vehicles_type.make_on_lane(self.road,
                                             ("a", "b", 1),
                                             longitudinal=56 + self.np_random.randn() * position_deviation,
                                             speed=14.5 + self.np_random.randn() * speed_deviation, id=vehicle_id)
        vehicle.plan_route_to('d')
        # vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)
        vehicle_id += 1

        # Vehicle 3: Blocking the ego vehicle
        vehicle = vehicles_type.make_on_lane(self.road,
                                             ("b", "c", 1),
                                             longitudinal=0.5 + self.np_random.randn() * position_deviation,
                                             speed=4.5 + self.np_random.randn() * speed_deviation, id=vehicle_id)
        vehicle.plan_route_to('d')
        # vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)
        vehicle_id += 1

        # Vehicle 4: Forcing risky overtake
        vehicle = vehicles_type.make_on_lane(self.road,
                                             ("b", "c", 0),
                                             longitudinal=17.5 + self.np_random.randn() * position_deviation,
                                             speed=5.5 + self.np_random.randn() * speed_deviation, id=vehicle_id)
        vehicle.plan_route_to('d')
        # vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)
        vehicle_id += 1

        # Vehicle 5: Blocking the ego vehicle
        vehicle = vehicles_type.make_on_lane(self.road,
                                             ("c", "d", 0),
                                             longitudinal=1 + self.np_random.randn() * position_deviation,
                                             speed=3.5 + self.np_random.randn() * speed_deviation, id=vehicle_id)
        vehicle.plan_route_to('d')
        # vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)
        vehicle_id += 1

        # Vehicle 6: Forcing risky overtake
        vehicle = vehicles_type.make_on_lane(self.road,
                                             ("c", "d", 1),
                                             longitudinal=30 + self.np_random.randn() * position_deviation,
                                             speed=5.5 + self.np_random.randn() * speed_deviation, id=vehicle_id)
        vehicle.plan_route_to('d')
        # vehicle.randomize_behavior()
        self.road.vehicles.append(vehicle)

        for v in self.road.vehicles:  # Prevent early collisions
            if v is not ego_vehicle and np.linalg.norm(v.position - ego_vehicle.position) < 10:
                self.road.vehicles.remove(v)


    def _road_test(self) -> None:
        """Test function Rodolfo"""
        # self.road = Road(network=RoadNetwork.straight_road_network(self.config["lanes_count"]),
        #                  np_random=self.np_random, record_history=self.config["show_trajectories"])
        net = RoadNetwork()

        lanes = self.lanes_count
        angle = 0
        length = 10000
        for lane in range(lanes):
            origin = np.array([0, lane * StraightLane.DEFAULT_WIDTH])
            end = np.array([length, lane * StraightLane.DEFAULT_WIDTH])
            rotation = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
            origin = rotation @ origin
            end = rotation @ end
            line_types = [LineType.CONTINUOUS_LINE if lane == 0 else LineType.STRIPED,
                          LineType.CONTINUOUS_LINE if lane == lanes - 1 else LineType.NONE]
            new_lane = StraightLane(origin, end, line_types=line_types)
            net.add_lane("0", "1", new_lane)

        last_lane = StraightLane.DEFAULT_WIDTH * lanes
        # Highway lanes
        ends = [150, 80, 80, 10000]  # Before, converging, merge, after
        c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
        y = [last_lane + StraightLane.DEFAULT_WIDTH, last_lane + 2 * StraightLane.DEFAULT_WIDTH]
        line_type = [[c, s], [n, c]]
        line_type_merge = [[c, s], [n, s]]

        new_lane = StraightLane([0, last_lane], [sum(ends[:2]), last_lane], line_types=[c, n], forbidden=True)
        net.add_lane("a", "b", new_lane)

        for i in range(2):
            net.add_lane("a", "b", StraightLane([0, y[i]], [sum(ends[:2]), y[i]], line_types=line_type[i]))
            net.add_lane("b", "c",
                         StraightLane([sum(ends[:2]), y[i]], [sum(ends[:3]), y[i]], line_types=line_type_merge[i]))
            net.add_lane("c", "d", StraightLane([sum(ends[:3]), y[i]], [sum(ends), y[i]], line_types=line_type[i]))

        # Merging lane
        amplitude = 3.25
        ljk = StraightLane([0, 6.5 + 4 + 4 + 4 + last_lane], [ends[0], 6.5 + 4 + 4 + 4 + last_lane], line_types=[c, c],
                           forbidden=True)
        lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
                       amplitude, 2 * np.pi / (2 * ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
        lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
                           line_types=[n, c], forbidden=True)
        net.add_lane("j", "k", ljk)
        net.add_lane("k", "b", lkb)
        net.add_lane("b", "c", lbc)
        road = Road(network=net, np_random=self.env.np_random, record_history=self.record_history)
        road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))

        pos = new_lane.position(sum(ends[:2]), 0)
        road.objects.append(Obstacle(road, pos))
        self.road = road

    def _vehicle_road_test(self) -> None:
        """Test function Rodolfo"""
        self.controlled_vehicles = []

        road = self.road
        ego_vehicle = self.env.action_type.vehicle_class(road,
                                                         road.network.get_lane(("a", "b", 1)).position(0, 0),
                                                         speed=30)
        road.vehicles.append(ego_vehicle)

        other_vehicles_type = utils.class_from_path(self.other_vehicles_type)

        # spawn vehicles in lane and possition
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(90, 0), speed=29))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(30, 0), speed=31))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(5, 0), speed=31.5))
        # road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(90, 0), speed=29))

        merging_v = other_vehicles_type(road, road.network.get_lane(("j", "k", 0)).position(110, 0), speed=20)
        merging_v.target_speed = 30
        road.vehicles.append(merging_v)
        # self.vehicle = ego_vehicle

        self.controlled_vehicles.append(ego_vehicle)
        self.road = road

        vehicles_type = utils.class_from_path(self.other_vehicles_type)

        # for _ in range(self.config["vehicles_count"]):
        #     self.road.vehicles.append(vehicles_type.create_random(self.road, spacing=1 / self.config["vehicles_density"]))

        # road.vehicles.append(other_vehicles_type(road,road.network.get_lane(("a", "b", 2)).position(100, 0), enable_lane_change=False,speed=31))
        density = 2
        stop_flag = False
        for i in range(6):
            if self.env.config['stopping_vehicle']['stop_flag']:
                stop_flag = True if i == self.env.config['stopping_vehicle']['id'] else False
            self.road.vehicles.append(
                CustomVehicleTurn.create_random_in_a_lane(self.road, ['0', '1', 1], config=self.env.config,
                                                          spacing=1 / density, speed=20, id=i))
        #
        for _ in range(6):
            self.road.vehicles.append(
                CustomVehicle.create_random_in_a_lane(self.road, ['0', '1', 2], config=self.env.config,
                                                      spacing=1 / density, speed=20))
        # for _ in range(self.config["controlled_vehicles_random"]):
        #     vehicle = self.action_type.vehicle_class.create_random(self.road,
        #                                                            speed=25,lane_id=self.config["initial_lane_id"],spacing=self.config["ego_spacing"]) #
        #     self.controlled_vehicles.append(vehicle)
        #     self.road.vehicles.append(vehicle)
