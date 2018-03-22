
import sys
import numpy as np
from collections import deque
from datetime import datetime

class StopWatch:
    def __init__(self):
        self.start_time = None

    def start(self):
        self.start_time = datetime.now().replace(microsecond=0)

    def get_time(self):
        if self.start_time is not None:
            return datetime.now().replace(microsecond=0) - self.start_time
        else: return None

class ProgressBar:
    def __init__(self, max_value, size=20, prefix='', suffix=''):
        self.max_value = max_value
        self.size = size
        self.current = 0
        self.prefix = str(prefix) + ' - '
        self.suffix = ' - ' + str(suffix)
        self.visible = False

    def set_visible(self, visible):
        self.visible = visible
        if visible:
            self.draw()

    def set(self, n=None, prefix=None, suffix=None):
        if n:
            self.current = n
        if prefix is not None:
            self.prefix = str(prefix) + ' - '
        if suffix is not None:
            self.suffix = ' - ' + str(suffix)
        if self.visible:
            self.draw()

    def step(self):
        self.current += 1
        if self.visible:
            self.draw()

    def calculate_percentage(self):
        self.percent = self.current / self.max_value

    def draw(self):
        self.visible = True
        self.calculate_percentage()
        print('\r{}[{}{}]( {:.1f}% ){}{}'.format(self.prefix, '#'*round(self.percent * self.size), ' '*round((1-self.percent)*self.size), 100*self.percent, self.suffix, ' '*10), end='', flush=True)

class Point:
    def __init__(self, lat, lng, t=0):
        self.arr = np.array([float(lat), float(lng)])
        self.t = int(t)

    @property
    def lat(self):
        return self.arr[0]

    @property
    def lng(self):
        return self.arr[1]

    def __str__(self):
        return '({0:.6f}, {1:.6f})'.format(self.lat, self.lng)

    def __eq__(self, other):
        return all(self.arr == other.arr) and self.t == other.t

    def distance(self, p):
        return np.linalg.norm(self.arr - p.arr)

class Statistics:

    def __init__(self, *args):
        self.statistics = { key : lambda:0 for key in args}

    def __setitem__(self, key, value):
        self.statistics[key] = lambda:value

    def __getitem__(self, key):
        if key not in self.statistics:
            print('{} not in statistics'.format(key))
        else:
            return self.statistics[key]()

    def add_lambda_statistics(self, new_key, func, key1, key2):
        self.statistics[new_key] = lambda: func(self[key1], self[key2])

    def save(self, filename):
        with open('statistics_' + filename, 'w') as f:
            for key in self.statistics:
                f.write('{} : {}\n'.format(key, self[key]))

class TrajectoryFixer:

    def __init__(self, spatial_limit = 0.002, time_limit = 30000, min_boundary = 0.01):
        self.spatial_limit = spatial_limit
        self.time_limit = time_limit
        self.min_boundary = min_boundary
        self.watch = StopWatch()
        self.progress_bar = None
        self.next_id = 0
        self.statistics = Statistics('n_points', 'mantained_points', 'n_of_trajectories', 'number_of_mantained_drivers', 'end_n_of_trajectories', 'total_time')
        self.statistics.add_lambda_statistics('trajectories_too_short', lambda x1, x2: x1-x2, 'n_of_trajectories', 'number_of_mantained_drivers')
        self.statistics.add_lambda_statistics('discarded_points', lambda x1, x2: x1-x2, 'n_points', 'mantained_points')
        self.statistics.add_lambda_statistics('discarded_points_proportion', lambda x1, x2: x1/x2, 'discarded_points', 'n_points')
        self.statistics.add_lambda_statistics('end_number_of_trajectories_proportion', lambda x1, x2: x1/x2, 'end_n_of_trajectories', 'n_of_trajectories')
        self.statistics.add_lambda_statistics('trajectories_too_short_proportion', lambda x1, x2: x1/x2, 'trajectories_too_short', 'n_of_trajectories')
        self.statistics.add_lambda_statistics('mean_time_between_points_before_processing', lambda x1, x2: x1/x2, 'total_time', 'n_points')
        self.statistics.add_lambda_statistics('mean_time_between_points_after_processing', lambda x1, x2: x1/x2, 'total_time', 'mantained_points')

    def line_count(self, filename):
        with open(filename) as f:
            return sum(1 for l in f) - 1

    def isvalid(self, traj):
        max_lat, min_lat, max_lng, min_lng = -np.inf, np.inf, -np.inf, np.inf
        for p in traj:
            max_lat, min_lat, max_lng, min_lng = max(max_lat, p.lat), min(min_lat, p.lat), max(max_lng, p.lng), min(min_lng, p.lng)

        # print(max_lat - min_lat, max_lng - min_lng)

        return max_lat - min_lat > self.min_boundary or max_lng - min_lng > self.min_boundary

    def fix_and_split(self, trajectory):
        l = deque(trajectory)

        fixed = []

        while l:
            new_traj = [l.popleft()]

            working_list = deque()
            while l and l[0].distance(new_traj[-1]) < self.spatial_limit and l[0].t > new_traj[-1].t and l[0].t - new_traj[-1].t < self.time_limit:
                working_list.append(l.popleft())

            while working_list:
                self.progress_bar.set(suffix=len(working_list), prefix=self.watch.get_time())
                # print(len(l))
                closest = min(working_list, key = lambda x: new_traj[-1].distance(x))

                while working_list and working_list[0].t <= closest.t:
                    working_list.popleft()

                while l and l[0].distance(new_traj[-1]) < self.spatial_limit and l[0].t > new_traj[-1].t and l[0].t - new_traj[-1].t < self.time_limit:
                    working_list.append(l.popleft())

                if new_traj[-1].distance(closest) > self.spatial_limit: 
                    break
                # print(closest)
                new_traj.append(closest)
            if self.isvalid(new_traj):
                fixed.append(new_traj)
        return fixed

    def process_and_save_trajectory(self, tid, trajectory, output_filename):
        trajectory = sorted(trajectory, key = lambda x: x.t)

        with open(output_filename, 'a') as f:

            for traj in self.fix_and_split(trajectory):
                for p in traj:
                    self.statistics['mantained_points'] += 1
                    f.write('{};{};{};{};{}\n'.format(tid, self.new_id, p.lat, p.lng, p.t))
                self.statistics['total_time'] += traj[-1].t - traj[0].t
                self.statistics['end_n_of_trajectories'] += 1
                self.new_id += 1

    def fix_trajectories(self, filename, output_filename=None):
        if not output_filename:
            output_filename = 'fixed_' + filename

        print('Fixing: "{}" => "{}"'.format(filename, output_filename))

        with open(output_filename, 'w') as f:
            f.write('driver_id;id;lat;lng;timestamp\n')
        self.new_id = 0

        n_points = self.line_count(filename)
        self.statistics['n_points'] = n_points

        self.progress_bar = ProgressBar(n_points, 40, prefix=self.watch.get_time())
        self.progress_bar.draw()
        self.watch.start()

        with open(filename, 'r') as f:

            lines = iter(f)
            next(lines)
            prev_id = None
            trajectory = []
            for line in lines:
                self.progress_bar.step()
                data_line = line.rstrip().split(';')
                new_id = data_line[0]
                if new_id != prev_id:
                    self.statistics['n_of_trajectories'] += 1
                    if trajectory and self.isvalid(trajectory):
                        self.statistics['number_of_mantained_drivers'] += 1
                        self.process_and_save_trajectory(prev_id, trajectory, output_filename)
                    trajectory = []
                    prev_id = new_id
                trajectory.append(Point(data_line[1], data_line[2], data_line[3]))

            self.statistics['n_of_trajectories'] += 1
            if trajectory and self.isvalid(trajectory):
                self.statistics['number_of_mantained_drivers'] += 1
                self.process_and_save_trajectory(prev_id, trajectory, output_filename)

        self.statistics.save(filename)
        print('Total time : {}'.format(self.watch.get_time()))

def main(filename):
    trajectory_fixer = TrajectoryFixer()
    trajectory_fixer.fix_trajectories(filename)

if __name__ == '__main__':
    main(sys.argv[1])

