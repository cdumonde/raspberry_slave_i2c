#!/usr/bin/env python 
#This lib allow to use the lcd shield on the Raspberry PI Cluster

from i2c_device import i2c_dev
import time

class display_shield(i2c_dev):
	"""docstring for display_shield"""
	def __init__(self, arg):
		i2c_dev.__init__(self, address, smbus)
		self.init()
		
	def init(self):
		#PCA init
		self._dev_write(0x04, [0x00])
		self._dev_write(0x05, [0x00])
		self._dev_write(0x06, [0x00])
		self._dev_write(0x07, [0x1C])
		#LCD init
		sleep(0.015)
		self.__dev_write(0x03, self._dev_read(0x03) & 0x1F)
		__write_instruction(0x30)
		__write_instruction(0x30)
		__write_instruction(0x30)
		__write_instruction(0x38)
		__write_instruction(0x0C)
		__write_instruction(0x01)
		__write_instruction(0x07)

	def status(self):
		return self._dev_read(0x01) & 0x1C

	def red_led_set(self):
		self._dev_write(0x03, self._dev_read(0x03) | 0x01)

	def red_led_unset(self):
		self._dev_write(0x03, self._dev_read(0x03) & 0xFE)

	def green_led_set(self):
		self._dev_write(0x03, self._dev_read(0x03) | 0x02)

	def green_led_unset(self):
		self._dev_write(0x03, self._dev_read(0x03) & 0xFD)

	def cursor_home(self):
		__write_instruction(0x02)

	def clear_display(self):
		__write_instruction(0x01)

	def display_shift(self):
		__write_instruction(0x1F)

	def display_write_char(self, char):
		data = ((char & 1) << 7) | ((char & 2) << 5) | ((char & 4) << 3) | ((char & 8) << 1) | ((char & 16) >> 1) | ((char & 32) >> 3) | ((char & 64) >> 5) | ((char & 128) >> 7)
		self._dev_write(0x03, self._dev_read(0x03) | 0x80)
		self._dev_write(0x02, [data])
		self._dev_write(0x03, self._dev_read(0x03) | 0x7F)

	def __write_instruction(self, instr):
		data = ((intr & 1) << 7) | ((intr & 2) << 5) | ((intr & 4) << 3) | ((intr & 8) << 1) | ((intr & 16) >> 1) | ((intr & 32) >> 3) | ((intr & 64) >> 5) | ((intr & 128) >> 7)
		self._dev_write(0x02, [data])
		sleep(0.004)
		__lcd_enable()
		sleep(0x004)

	def __lcd_enable(self):
		self._dev_write(0x03, self._dev_read(0x03) | 0x20)
		sleep(0.001)
		self._dev_write(0x03, self._dev_read(0x03) & 0xDF)
