class MissionFactory():

    def __init__(self, env):
        self.env = env
        self.mission_accomplished = self.env.mission_accomplished
        self.mission_vehicle = self.env.mission_vehicle

    def check_mission_accomplished(self, mission_type):
        if mission_type == 'merging':
            if not self.mission_accomplished:
                self.mission_accomplished, self.mission_vehicle = self.check_merging_vehicle()

            return self.mission_accomplished, self.mission_vehicle

        if mission_type == 'exit':
            # return None, None
            if not self.mission_accomplished:
                self.mission_accomplished, self.mission_vehicle = self.check_exit_vehicle()

            return self.mission_accomplished, self.mission_vehicle

        if mission_type == 'none':
            return False, None

    def check_merging_vehicle(self):
        # TODO
        vehicle = None
        for vehicle in self.env.road.vehicles:
            if vehicle.id == self.env.config['merging_vehicle']['id']:
                _from, _to, _id = vehicle.target_lane_index
                right_lane = len(self.env.road.network.graph['b']['c']) - 1
                if _from == 'b' and _to == 'c' and _id != right_lane:
                    return True, vehicle

        return False, vehicle

    def check_exit_vehicle(self):
        # TODO
        vehicle = None
        for vehicle in self.env.road.vehicles:
            if vehicle.id == self.env.config['exit_vehicle']['id']:
                _from, _to, _id = vehicle.target_lane_index
                right_lane = len(self.env.road.network.graph[_from][_to])-1

                if _from == '1' and _to == '2' and _id == right_lane:
                    return True, vehicle
                elif _from == '0' and _to == '1' and _id == right_lane:
                    return True, vehicle
                elif _id == right_lane:
                    return True, vehicle


        return False, vehicle