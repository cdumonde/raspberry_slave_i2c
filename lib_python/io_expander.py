#!/usr/bin/env python 
#This lib allow to use the temperature sensor on the Raspberry PI Cluster

import i2c_device

class io_expander(i2c_device):
	"""docstring for io_expander"""
	def __init__(self, address, smbus):
		super(io_expander, self).__init__(address, smbus)
		self.__pin = 0xFF

	def status(self):
		return self.__pin

	def pin_set(self, index):
		self.__pin |= 1 << index
		self.__dev_write(self, 0x00, self.__pin) 

	def pin_unset(self, index):
		self.__pin &= 0 << index
		self.__dev_write(self, 0x00, self.__pin) 

	def toogle(self):
		self.__pin = ~self.__pin
		self.__dev_write(self, 0x00, self.__pin)

	def pin_set_all(self):
		self.__pin = 0xFF
		self.__dev_write(self, 0x00, self.__pin)

	def pin_reset(self):
		self.__pin = 0x00
		self.__dev_write(self, 0x00, self.__pin)
