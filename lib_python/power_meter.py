#!/usr/bin/env python 
#This lib allow to use the power meter on the Raspberry PI Cluster

import i2c_device

class power_meter(i2c_device):
	"""docstring for power_meter"""
	def __init__(self, address, smbus):
		super(power_meter, self).__init__(address, smbus)
		self.init(self)

	def init(self):
		if self.__dev_write(self, 0x00, [0x00, 0x80]) == -1 || self.__dev_write(self, 0x00, [0x9F, 0x09]) == -1:
			print ("power meter initialisation failed\n")

	def get_power(self):
		return self.__dev_read(self, 0x03)

	def get_current(self):
		return self.__dev_read(self, 0x04)

	def get_voltage(self):
		return self.__dev_read(self, 0x01)