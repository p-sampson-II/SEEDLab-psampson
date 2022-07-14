import i2c

while True:
    i2c.command(i2c.CMD_TURN, float(3.14159/8))
