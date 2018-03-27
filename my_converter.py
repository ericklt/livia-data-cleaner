import sys

class Point:
    def __init__(self, lat, lng, t):
        self.lat = float(lat)
        self.lng = float(lng)
        self.t = int(t)

    def __str__(self):
        return '{};{}'.format(self.lat, self.lng)

def save_trajectory(output, tid, trajectory):
    line = [tid] + [str(p) for p in sorted(trajectory, key= lambda p: p.t)]
    output.write(';'.join(line))
    output.write('\n')

def main(filename):

    output_filename = 'converted_' + filename

    print('Converting: "{}" => "{}"'.format(filename, output_filename))

    with open(filename, 'r') as f:
        with open(output_filename, 'w') as output:

            trajectory = []
            current_id = None
            for line in f:
                if line.startswith(('driver', 'id')):
                    continue
                data_line = line.split(';')
                if not current_id:
                    current_id = data_line[1]
                new_id = data_line[1]
                if new_id != current_id:
                    save_trajectory(output, current_id, trajectory)
                    trajectory = []
                trajectory.append(Point(data_line[2], data_line[3], data_line[4]))
                current_id = new_id
            save_trajectory(output, current_id, trajectory)


if __name__ == '__main__':
    main(sys.argv[1])
