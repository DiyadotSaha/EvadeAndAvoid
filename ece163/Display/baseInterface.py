import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets
import sys
from . import vehicleDisplay
from ..Containers import States
import math
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D

import traceback
import os
import datetime
import random


simulationThreadRate = 20

class aboutTab(QtWidgets.QWidget):
	def __init__(self, parent=None):
		super().__init__(parent)

		usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(usedLayout)
		textLocation = os.path.join(sys.path[0], 'about.html')
		try:
			with open(textLocation) as f:
				aboutText = f.read()
		except FileNotFoundError:
			aboutText = "No About.html file found"
		aboutText = QtWidgets.QLabel(aboutText)
		aboutText.setWordWrap(True)
		usedLayout.addWidget(aboutText)
		usedLayout.addStretch()

class exceptionTab(QtWidgets.QWidget):
	def __init__(self, parent=None):
		super().__init__(parent)

		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)

		self.usedLayout.addWidget(QtWidgets.QLabel("Last Exception simulation had will appear here"))

		self.exceptionText = QtWidgets.QPlainTextEdit("None")
		self.exceptionText.setReadOnly(True)
		self.usedLayout.addWidget(self.exceptionText)
		return

	def setExceptionText(self, newText):
		self.exceptionText.clear()
		self.exceptionText.appendPlainText(newText)

class baseInterface(QtWidgets.QMainWindow):
	updateVehiclePositionSignal = QtCore.pyqtSignal(list)  # signal for redrawing the vehicle

	def __init__(self, parent = None):
		"""
		important elements

		self.stateUpdateDefList contains a list of function pointers that subscribe to state changes
		runUpdate() overwritten to actually perform one step of the simulation
		resetSimulationActions() actions taken on simulation reset
		getvehicleState() overwritten to always return the valid state
		"""
		super().__init__(parent)

		# before we set up the gui we need a few data types to store
		self.simulationPaused = True

		self.curSpeedRate = 1
		self.newSpeedRate = 1
		self.threadTick = 0
		self.curMultiTickRate = 1
		self.newMultiTickRate = 1
		self.bombTickRate = 1
		self.bombTickCount = 0

		self.stateSet = False

		# we need a set of callbacks for when the state changes and the various gui elements need updating
		self.stateUpdateDefList = list()

		# after updates these are ones that take no arguments but same idea
		self.afterUpdateDefList = list()

		# we need to start the thread now, it just runs
		self.simulationTimedThread = QtCore.QTimer()
		self.simulationTimedThread.setInterval(simulationThreadRate)
		self.simulationTimedThread.timeout.connect(self.runSimulation)

		self.mainWidget = QtWidgets.QWidget()
		self.mainLayout = QtWidgets.QVBoxLayout()
		self.setCentralWidget(self.mainWidget)
		self.mainWidget.setLayout(self.mainLayout)

		self.mainSplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
		self.mainLayout.addWidget(self.mainSplitter, 10)

		self.vehicleDisplaySplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
		# self.vehicleDisplaySplitter.setFrameStyle(QFrame.Box)
		self.mainSplitter.addWidget(self.vehicleDisplaySplitter)



		#IMPORTANT THIS IS WHERE THE ADD BOMB FUNCTIONALITY IS TO THE GUI
		self.vehicleInstance = vehicleDisplay.vehicleDisplay()
		self.vehicleDisplaySplitter.addWidget(self.vehicleInstance)
		self.stateUpdateDefList.append(self.vehicleInstance.updateVehiclePosition)


		self.inputBaseWidget = QtWidgets.QWidget()
		self.vehicleDisplaySplitter.addWidget(self.inputBaseWidget)
		self.vehicleDisplaySplitter.setStretchFactor(0, 1)
		self.inputLayout = QtWidgets.QVBoxLayout()
		self.inputBaseWidget.setLayout(self.inputLayout)
		self.inputTabs = QtWidgets.QTabWidget()
		self.inputLayout.addWidget(self.inputTabs)
		# self.inputLayout.addWidget(QLabel("Input Here"))

		self.outPutWidget = QtWidgets.QWidget()
		self.outPutLayout = QtWidgets.QVBoxLayout()
		self.outPutWidget.setLayout(self.outPutLayout)
		self.mainSplitter.addWidget(self.outPutWidget)
		self.mainSplitter.setStretchFactor(0, 4)
		self.mainSplitter.setStretchFactor(1, 3)

		self.outPutTabs = QtWidgets.QTabWidget()
		self.outPutTabs.addTab(aboutTab(), 'About')
		self.exceptionTab = exceptionTab()
		self.outPutTabs.addTab(self.exceptionTab, 'Last Exception')
		self.exceptionTabIndex = self.outPutTabs.count()-1

		self.outPutLayout.addWidget(self.outPutTabs)

		self.numericStateBox = QtWidgets.QHBoxLayout()

		self.outPutLayout.addLayout(self.numericStateBox)

		self.numericStateGrid = QtWidgets.QGridLayout()
		self.numericStateBox.addLayout(self.numericStateGrid)
		self.numericStatesDict = dict()
		for i, name in enumerate(['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll']):
			newLabel = QtWidgets.QLabel("{}: ".format(name))
			self.numericStateGrid.addWidget(newLabel, 0, i)
			self.numericStatesDict[name] = newLabel
		self.stateUpdateDefList.append(self.updateNumericStateBox)

		# self.numericStateBox.addStretch()

		self.simulationControlsBox = QtWidgets.QHBoxLayout()
		self.mainLayout.addLayout(self.simulationControlsBox, 0)
		self.simulationControlsBox.addWidget(QtWidgets.QLabel("Simulation Controls")) # temp text for layout

		self.playButton = QtWidgets.QPushButton("Play")
		self.pauseButton = QtWidgets.QPushButton("Pause")
		self.resetButton = QtWidgets.QPushButton("Reset")

		self.simulationControlsBox.addWidget(self.playButton)
		self.simulationControlsBox.addWidget(self.pauseButton)
		self.simulationControlsBox.addWidget(self.resetButton)

		self.playButton.clicked.connect(self.PlaySimulation)
		self.pauseButton.clicked.connect(self.PauseSimulation)
		self.resetButton.clicked.connect(self.ResetSimulation)

		self.pauseButton.setDisabled(True)

		self.simulationSpeedsBox = QtWidgets.QHBoxLayout()
		self.simulationControlsBox.addLayout(self.simulationSpeedsBox)
		self.simulationSpeedsGroup = QtWidgets.QButtonGroup()

		for ratio in [20, 8, 4, 2, 1, 1/2, 1/4, 1/8]:
			newRatio = 1/(simulationThreadRate/(simulationThreadRate/ratio))
			newRadio = QtWidgets.QRadioButton("{}x".format(newRatio))
			newRadio.threadRate = ratio
			self.simulationSpeedsBox.addWidget(newRadio)
			self.simulationSpeedsGroup.addButton(newRadio)
		self.simulationSpeedsGroup.buttons()[-4].setChecked(True)
		self.simulationSpeedsGroup.buttonToggled.connect(self.speedChangedResponse)

		self.currentTime = 0
		self.simulationControlsBox.addWidget(QtWidgets.QLabel("Current Time: "))
		self.currentTimeLabel = QtWidgets.QLabel(str(datetime.timedelta(seconds=self.currentTime)))
		self.simulationControlsBox.addWidget(self.currentTimeLabel)
		self.simulationControlsBox.addStretch()

		self.mainLayout.addStretch()

		self.afterUpdateActions()
		return

	def runSimulation(self):
		"""
		internal method called in a thread to handle simulation updates
		"""
		if self.threadTick % self.curSpeedRate == 0:
			try:
				for i in range(self.curMultiTickRate):
					self.runUpdate()
					self.runUpdate()  # Only updating the screen at half of the integration rate
			except Exception as e:
				self.raiseExceptionToUser(traceback.format_exc())
				# traceback.print_exc()
				self.PauseSimulation()
			self.afterUpdateActions()
			try:
				self.currentTime = self.simulateInstance.time
				self.currentTimeLabel.setText(str(datetime.timedelta(seconds=self.currentTime)))
			except AttributeError:
				pass
			self.curSpeedRate = self.newSpeedRate
			self.curMultiTickRate = self.newMultiTickRate
		self.threadTick += 1
		return

	def speedChangedResponse(self, checked):
		if checked.isChecked():
			if checked.threadRate >= 1:
				self.newSpeedRate = checked.threadRate
				self.newMultiTickRate = 1
			else:
				self.newSpeedRate = 1
				self.newMultiTickRate = int(1/checked.threadRate)
				print(self.newMultiTickRate)
		return


	def PlaySimulation(self):
		"""
		starts simulation thread if stopped
		"""
		print('play')
		self.playButton.setDisabled(True)
		self.pauseButton.setDisabled(False)
		self.simulationPaused = False
		self.simulationTimedThread.start()
		
		return

	def PauseSimulation(self):
		"""
		pauses simulation thread if playing
		"""
		print('pause')
		n = []
		e = []
		d = []
		b_rad = self.vehicleInstance.graph_bombs[0].max_radius
		bombn = []
		bombe = []
		bombd = []
		fig = plt.figure()
		ax = plt.axes(projection='3d')
		print("Printing out the bombs: ")
		for bomb in self.vehicleInstance.graph_bombs:
			print("bomb-origin: ", bomb.bn, bomb.be, bomb.bd)
			bombn += [bomb.bn]
			bombe += [bomb.be]
			bombd += [bomb.bd]
		for point in self.vehicleInstance.planeTrailPoints:
			n += [point[0]]
			e += [point[1]]
			d += [point[2]]
		for x, y, z in zip(bombn, bombe, bombd):
			u = np.linspace(0, 2 * np.pi, 30)
			v = np.linspace(0, np.pi, 30)
			x_sphere = b_rad * np.outer(np.cos(u), np.sin(v)) + x
			y_sphere = b_rad * np.outer(np.sin(u), np.sin(v)) + y
			z_sphere = b_rad * np.outer(np.ones(np.size(u)), np.cos(v)) + z
			ax.plot_surface(x_sphere, y_sphere, z_sphere, color='r', alpha=0.5)
		ax.plot(n, e, d)
		legend_elements = [Line2D([0], [0], marker='o', color='w', label='Explosion',
								  markerfacecolor='r', markersize=10, alpha=0.5),
						   Line2D([0], [0], color='w', label='Data',
								  markerfacecolor='b', markersize=15)]
		ax.legend(handles=legend_elements)
		ax.set_xlabel("N")
		ax.set_ylabel("E")
		ax.set_zlabel("D")
		ax.set_title("Plane's position in relation to the explosions")
		plt.show()
		self.playButton.setDisabled(False)
		self.pauseButton.setDisabled(True)
		self.simulationPaused = True
		self.simulationTimedThread.stop()
		return

	def ResetSimulation(self):
		"""
		pauses simulation and calls reset function
		"""
		self.playButton.setDisabled(False)
		self.pauseButton.setDisabled(True)
		self.simulationPaused = True
		self.simulationTimedThread.stop()
		self.vehicleInstance.reset()
		self.resetSimulationActions()
		# self.updateGuiStateElements(vehicleDisplay)
		self.currentTime = 0
		self.currentTimeLabel.setText(str(datetime.timedelta(seconds=self.currentTime)))
		print('reset')
		return

	def resetSimulationActions(self):
		"""
		this def needs to be filled with all actions taken to reset the simulation. Does nothing without subclassing.
		"""
		return
	

	def afterUpdateActions(self):
		"""
		Internal method to run all the state updates with gui elements, generally not called directly.
		"""
		curState = self.getVehicleState()
		for updater in self.stateUpdateDefList:
			updater(curState)
		for updater in self.afterUpdateDefList:
			updater()

		
		
		if(self.stateSet == True):
			self.updateBombs(self.vehicleState)
		
		
		return
	
	def getExplosions(self):
		return self.vehicleInstance.getExplosions()
	
	def setState(self, vehicleState):
		
		self.vehicleState = vehicleState
		self.stateSet = True

	def updateBombs(self, state):

		self.bombTickCount += self.bombTickRate
		if(self.bombTickCount >= random.randrange(60, 200, 10)):
			self.bombTickCount = 0
			print(self.vehicleState.pn, self.vehicleState.pe, self.vehicleState.pd)
			x = self.vehicleState.pe + random.randrange(-30, 30, 10)
			y = self.vehicleState.pn + random.randrange(-30, 30, 10)
			z = -self.vehicleState.pd + random.randrange(-30, 30, 5)

			print("spawining bomb at x,y,z = ", x,y,z)
			self.vehicleInstance.addBomb(x, y, z)		
		self.vehicleInstance.updateBomb()



	def runUpdate(self):
		"""
		Method that actually handles whatever update call is needed. Does nothing without a subclass.
		"""
		return

	def getVehicleState(self):
		"""
		Returns the current vehicle state for gui element updates. Overwrite for something useful.
		"""
		return States.vehicleState()

	def updateNumericStateBox(self, newState):
		"""
		simply updates the numeric state box and used as an example of an state update def

		:param newState: an instance of vehicleState
		"""
		for key, label in self.numericStatesDict.items():
			newVal = float(getattr(newState, key))
			if key in ['yaw', 'pitch', 'roll']:
				newVal = math.degrees(newVal)
			label.setText("{}: {:03.4}".format(key, newVal))
		return

	def raiseExceptionToUser(self, exceptionText):
		self.exceptionTab.setExceptionText(exceptionText)
		self.outPutTabs.setCurrentIndex(self.exceptionTabIndex)

		return