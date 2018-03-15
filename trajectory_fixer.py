
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

def read_trajectories(filename):
    print('Reading: ', filename)

    trajectories = {}
    with open(filename, 'r') as f:
        lines = f.readlines()

        for line in lines[1:]:
            data_line = line.split(';')
            new_id = data_line[0]
            if new_id not in trajectories:
                trajectories[new_id] = []
            trajectories[new_id].append(Point(data_line[1], data_line[2], data_line[3]))
    return trajectories

def save_trajectories(output_filename, trajectories):
    print('Saving: ', output_filename)
    with open(output_filename, 'w') as f:
        f.write('id;lat;lng;timestamp\n')
        for _id in trajectories:
            for p in trajectories[_id]:
                f.write('{};{};{};{}\n'.format(str(_id), p.lat, p.lng, p.t))

class TrajectoryFixer:

    def __init__(self, spatial_limit = 0.002, time_limit = 30000, min_boundary = 0.01):
        self.spatial_limit = spatial_limit
        self.time_limit = time_limit
        self.min_boundary = min_boundary
        self.watch = StopWatch()
        self.progress_bar = None

    def order_by_timestamp(self, trajectories):
        for t in trajectories:
            trajectories[t] = sorted(trajectories[t], key = lambda x: x.t)

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

    def fix_trajectories(self, trajectories):
        self.order_by_timestamp(trajectories)
        new_trajectories = {}
        counter = 0

        key_list = [key for key in trajectories if self.isvalid(trajectories[key])]

        self.progress_bar = ProgressBar(len(key_list), 40, prefix=self.watch.get_time())
        self.progress_bar.draw()

        self.watch.start()

        for _id in key_list:

            counter += 1
            self.progress_bar.set(counter)

            for traj in self.fix_and_split(trajectories[_id]):
                new_trajectories[len(new_trajectories)] = traj

        print('Total time : {}'.format(self.watch.get_time()))

        return new_trajectories

def main(filemane):

    trajectories = read_trajectories(filename)

    trajectory_fixer = TrajectoryFixer()

    new_trajectories = trajectory_fixer.fix_trajectories(trajectories)

    save_trajectories('fixed_' + filename, new_trajectories)

if __name__ == '__main__':
    main(sys.argv[1])

