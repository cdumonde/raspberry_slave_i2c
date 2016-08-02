#!/usr/bin/env python 
#This lib allow to use the temperature sensor on the Raspberry PI Cluster

import i2c_device

class temp_sensor(i2c_device):
	"""docstring for temp_sensor"""
	def __init__(self, address, smbus):
		super(temp_sensor, self).__init__(address, smbus)
		self.__init(self)

	def __init(self):
		if self.__dev_write(self, 0x01, 0x00) == -1:
			print ("temperature sensor initialisation failed\n")

	def get_temp(self):
		temp = self.__dev_read(self, 0x00)
		temp = (temp & 0xFF80)
		if temp != (temp & 0x7FFF):
			temp = -(temp >> 8)
		else:
			temp = temp>>8
		return temp