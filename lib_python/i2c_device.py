#!/usr/bin/env python 
#This file represent the i2c_device class and property

import smbus

class i2c_dev:
	"""docstring for i2c_dev"""
	def __init__(self, address, smbus):
		self.address = address
		self.bus = smbus

	def __dev_read(self, reg_data):
		try:
			return self.bus.read_i2c_block_data(i2c_dev.address, reg_data)
		except:
			return -1

	def __dev_write(self, reg_data, data):
		try:
			return self.bus.write_i2c_block_data(i2c_dev.address, reg_data, data)
		except:
			return -1