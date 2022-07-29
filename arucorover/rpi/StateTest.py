from statemachine import StateMachine, State
import i2c
import math
import threading

SEEK_ANGLE = math.pi/8
FAKE_CAMERA_ANGLE = math.pi/5
FWD_SPEED = 1.4
DEAD_RECON_DISTANCE = 0.1

class VirtualRover(StateMachine):
    seek = State('Seek',initial=True)
    correct = State('Angle Correction')
    forward = State('Move Forward')
    moveOver = State('Move Over Marker')
    stop = State('Stop Moving')

    noMarker = seek.to(seek)
    enter = seek.to(correct)
    exit = correct.to(seek)
    center = correct.to(forward)
    side = forward.to(correct)
    bottom = forward.to(moveOver)
    cont = moveOver.to(seek)
    end = moveOver.to(stop)

    markerID = 1

    def on_enter_seek(self):
        print('Making a PI/8 rad turn.')
        i2c.command(i2c.CMD_TURN, SEEK_ANGLE)
        if isMarker:
            self.enter()
        else:
            self.noMarker()

    def on_enter_correct(self):
        print('Centering Aruco Marker in camera\'s view.')
        i2c.command(i2c.CMD_TURN, FAKE_CAMERA_ANGLE)

    def on_enter_forward(self):
        print('Moving towards Aruco Marker.')
        i2c.command(i2c.CMD_VELOC, FWD_SPEED)

    def on_enter_moveOver(self):
        print('Moving over Aruco Marker.')
        i2c.command(i2c.CMD_POS, DEAD_RECON_DISTANCE)

    def on_enter_stop(self):
        print('Haulting rover.')
        i2c.command(0xFF, 0x04)
