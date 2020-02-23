## This is to increase the size of obstacle in the map

import numpy as np

def modify(map, radius): ## radius represents the robot radius
	print("Map Processing...")
	def modifySurroundings(obstacle, rr, free):
		(xo, yo) = obstacle
		x_min, x_max, y_min, y_max = xo-rr, xo+rr, yo-rr, yo+rr

		x_range = np.arange(x_min, x_max+1)
		y_range = np.arange(y_min, y_max+1)

		for x in x_range:
			for y in y_range:
				inrange = x <= width-1 and x >= 0 and y <= height-1 and y >= 0
				if (inrange and map[x][y] == 0 and ((x-xo)**2 + (y-yo)**2)**0.5 <= rr):
					map[x][y] = 999


	width = map.shape[0]
	height = map.shape[1]
	free = []
	frontiers = []

	count_explore = 0
	for x in range(width):
		for y in range(height):
			if (map[x][y] != -1):
				count_explore += 1

			if (map[x][y] == 100):
				modifySurroundings((x,y), radius, free)


	for x in range(width):
		for y in range(height):
			if (map[x][y] == 0):
				free.append((x, y))
			if (map[x][y] == -1):
				directions = [(x+1, y), (x-1, y), (x, y+1), (x, y-1), (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)]
				for direction in directions:
					(dx, dy) = direction
					inrange = dx <= width-1 and dx >= 0 and dy <= height-1 and dy >= 0
					if inrange and map[dx][dy] == 0:
						frontiers.append((x, y))
						break


	print('Map Processing Finished')
	return map, free, frontiers, count_explore


def main():
	current_map = np.load('map.npy')
	#modified_map = modify(current_map, 1)
	modified_map0, free0 = modify(current_map, 0)
	modified_map, free = modify(current_map, 7)
	print(len(free0))
	print(len(free))
	#np.savetxt('modified_map_r7.csv', modified_map, delimiter = ",")
	print(modified_map.shape)





if __name__ == "__main__":
	main()