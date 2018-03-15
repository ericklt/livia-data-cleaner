import sys

class Point:
    def __init__(self, lat, lng, t):
        self.lat = float(lat)
        self.lng = float(lng)
        self.t = int(t)

    def __str__(self):
        return '{};{}'.format(self.lat, self.lng)

def main(filename):

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

    for _id in trajectories:
        trajectories[_id].sort(key=lambda p : p.t)

    print('{}, {}'.format(max(trajectories.keys()), min(trajectories.keys())))

    output_filename = 'converted_' + filename
    print('Saving: ', output_filename)

    with open(output_filename, 'a') as f:
        for _id in trajectories:
            line = [_id] + [str(p) for p in trajectories[_id]]
            f.write(';'.join(line))
            f.write('\n')

if __name__ == '__main__':
    main(sys.argv[1])
