#!/usr/bin/env python 
#This lib allow to use the temperature sensor on the Raspberry PI Cluster

from i2c_device import i2c_dev

class io_expander(i2c_device):
	"""docstring for io_expander"""
	def __init__(self, address, smbus):
		i2c_dev.__init__(self, address, smbus)
		self.init()

	def init(self):
		self.__pin = 0x00
		self._dev_write(0x00, [self.__pin]) 

	def status(self):
		return self.__pin

	def pin_set(self, index):
		self.__pin |= 1 << index
		self._dev_write(0x00, [self.__pin]) 

	def pin_unset(self, index):
		self.__pin &= 0 << index
		self._dev_write(0x00, [self.__pin]) 

	def toogle(self):
		self.__pin = ~self.__pin
		self._dev_write(0x00, [self.__pin])

	def pin_set_all(self):
		self.__pin = 0xFF
		self._dev_write(0x00, [self.__pin])

	def pin_reset(self):
		self.__pin = 0x00
		self._dev_write(0x00, [self.__pin])
