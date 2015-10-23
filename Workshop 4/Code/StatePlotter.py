#Records The data of some system, and outputs it to a file at some point.

import matplotlib.pyplot as plt

class StatePlotter:
	def __init__(self, timestep):
		self.datapoints = []
		self.timepoints = []
		self.timestep = timestep
		self.controlpoints = []

	def recordDataPoint(self, data): #data can be a list of lenght n. each element is a point in a different dataset
		if len(self.datapoints) == 0:
			for i in data:
				self.datapoints.append([i[0]])
		else:
			for i in range(len(data)):
				self.datapoints[i].append(data[i][0])
		if len(self.timepoints) == 0:
			self.timepoints.append(0)
		else:
			self.timepoints.append(self.timepoints[-1] + self.timestep)
		
	def recordControlSignal(self, control):
		if len(self.controlpoints) == 0:
			for i in control:
				self.controlpoints.append([i[0]])
		else:
			for i in range(len(control)):
				self.controlpoints[i].append(control[i][0])

	def plotData(self, outputfile_state, outputfile_phase, sizex = 30, sizey = 15): #writes the plot to a file, with the path given in outputfile
		f = plt.figure(figsize = (sizex, sizey))
		i = 1
		for datalist in self.datapoints:
			plt.plot(self.timepoints, datalist, label = "$x_" + str(i) + "$")
			i += 1
		i = 1
		for controllist in self.controlpoints:
			plt.plot(self.timepoints, controllist, label = "$u_" + str(i) + "$")
			i += 1
		plt.legend()
		plt.savefig(outputfile_state)
		f = plt.figure(figsize = (sizex, sizey))
		plt.plot(self.datapoints[0], self.datapoints[1])
		plt.savefig(outputfile_phase)
