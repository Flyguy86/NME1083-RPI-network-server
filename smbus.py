"""
Shim module so libraries that expect 'import smbus' can use smbus2 instead.

py_qmc5883l imports 'smbus.SMBus', and smbus2 provides a drop-in compatible SMBus class.
"""

from smbus2 import SMBus, i2c_msg  # re-export typical API symbols

__all__ = ["SMBus", "i2c_msg"]
