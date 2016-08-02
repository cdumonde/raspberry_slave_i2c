#!/usr/bin/env python 
#This lib allow to use the power meter on the Raspberry PI Cluster

from i2c_device import i2c_dev

class power_meter(i2c_device):
	"""docstring for power_meter"""
	def __init__(self, address, smbus):
		i2c_dev.__init__(self, address, smbus)
		self.init(self)

	def init(self):
		if self._dev_write(0x00, [0x00, 0x80]) == -1 || self.__dev_write(self, 0x00, [0x9F, 0x09]) == -1:
			print ("power meter initialisation failed\n")

	def get_power(self):
		return self._dev_read(0x03)

	def get_current(self):
		return self._dev_read(0x04)

	def get_voltage(self):
		return self._dev_read(0x01)