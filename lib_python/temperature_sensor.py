#!/usr/bin/env python 
#This lib allow to use the temperature sensor on the Raspberry PI Cluster

from i2c_device import i2c_dev

class temp_sensor(i2c_device):
	"""docstring for temp_sensor"""
	def __init__(self, address, smbus):
		i2c_dev.__init__(self, address, smbus)
		self.init(self)

	def init(self):
		if self._dev_write(0x01, [0x00]) == -1:
			print ("temperature sensor initialisation failed\n")

	def get_temp(self):
		temp = (self._dev_read(0x00))[0]
		temp = (temp & 0xFF80)
		if temp != (temp & 0x7FFF):
			temp = -(temp >> 8)
		else:
			temp = temp>>8
		return temp