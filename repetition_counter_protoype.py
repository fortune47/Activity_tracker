def smoothen(pattern):

	batch_size = 5
	smooth_pattern = ""

	for i in range(0, len(pattern), batch_size):
		sample = pattern[i : i + batch_size]
		smooth_pattern += max(sample, key = sample.count)	

	return smooth_pattern


def calculate_reps(data):

	batch = 5
	tolerance = 0.02

	trend = ""

	for i in range(0, len(data), batch):

		curr_batch = data[i : i + batch]
		vote = []

		for j in range(0, len(curr_batch)):
			if curr_batch[j] < 0:
				vote.append(1)
			else:
				vote.append(-1)

		if len(vote) > 0 and max(vote, key = vote.count) == -1:
			trend += "U"
		else:
			trend += "L"

	print smoothen(trend)

if __name__ == "__main__":

	with open("rep3.txt", "r") as f:

		data_points = f.readlines()
	
		x = []
		y = []
		z = []

		for point in data_points:
			i, j, k = point.split("\t")
			x.append(float(i))
			y.append(float(j))
			z.append(float(k))

		print calculate_reps(x)
