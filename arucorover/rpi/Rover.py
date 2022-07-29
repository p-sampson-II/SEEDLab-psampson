from statemachine import StateMachine, State
import i2c
import math
import Global

class RoverState(State):
    def __init__(self, name, cmdIdent, cmdVal, value=None, initial=False):
        State.__init__(self,name,value,initial)
        self.cmdIdent = cmdIdent
        self.cmdVal = cmdVal

class VirtualRover(StateMachine):
    SEEK_ANGLE = math.pi/8
    FWD_SPEED = 1.4
    DEAD_RECON_DISTANCE = 0.1

    seek = State('Seek',initial=True)
    correct = State('Angle Correction')
    forward = State('Move Forward')
    moveOver = State('Move Over Marker')
    stop = State('Stop Moving')

    noMarker = seek.to(seek)
    enter = seek.to(correct)
    exit = correct.to(seek)
    retryCenter = correct.to(correct)
    center = correct.to(forward)
    onward = forward.to(forward)
    side = forward.to(correct)
    bottom = forward.to(moveOver)
    cont = moveOver.to(seek)
    end = moveOver.to(stop)

    def on_enter_seek(self):
        print('Making a PI/8 rad turn.')
        Global.cmd = (i2c.CMD_TURN,SEEK_ANGLE)
        if Global.isMarker == True:
            self.next = self.enter
        else:
            self.next = self.noMarker

    def on_enter_correct(self):
        print('Centering Aruco Marker in camera\'s view.')
        Global.cmd = (i2c.CMD_TURN,Global.camAngle)
        if Global.isCentered == True:
            self.next = self.center
        elif Global.isMarker == False:
            self.next = self.exit
        else:
            self.next = self.retryCenter

    def on_enter_forward(self):
        print('Moving towards Aruco Marker.')
        Global.cmd = (i2c.CMD_VELOC,Global.FWD_SPEED)
        if Global.isExitBottom == True:
            self.next = self.bottom
        elif Global.isCentered == False:
            self.next = self.side
        else:
            self.next = self.onward

    def on_enter_moveOver(self):
        print('Moving over Aruco Marker.')
        Global.cmd = (i2c.CMD_POS,DEAD_RECON_DISTANCE)
        if markerID >= maxMarkerID:
            self.next = self.stop
        else:
            self.next = self.cont

    def on_enter_stop(self):
        print('Haulting rover.')
        Global.cmd = (0xFF,0x04)
