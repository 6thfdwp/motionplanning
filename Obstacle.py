from shapely.geometry import *

class Obstacle:
    def __init__(self, coords):
        minx, miny = coords[0]
        maxx, maxy = coords[2]
        self.coords = coords
        self.rect = box(minx, miny, maxx, maxy)

    def collide(self, state):
        return self.rect.intersects(state.shape())

    def __str__(self):
        for i, each in enumerate(self.coords):
            if i == 0:
                str = '%.3f %.3f' % (each[0],each[1])
                continue
            str += ' %.3f %.3f' % (each[0],each[1])
        return str


if __name__ == '__main__':
    coords = [(0.2,0.3), (0.4,0.3), (0.4,0.5), (0.2,0.5)]
    obs = Obstacle(coords)
    print obs
