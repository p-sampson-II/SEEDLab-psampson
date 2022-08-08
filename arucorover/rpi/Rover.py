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
    SEEK_ANGLE = math.pi/16
    FWD_SPEED = 1.4
    DEAD_RECON_DISTANCE = 0.2

    take = 0

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
    whereDidHeGo = forward.to(seek)
    cont = moveOver.to(seek)
    end = moveOver.to(stop)

    def __init__(self):
        StateMachine.__init__(self)
        self.next = self.noMarker

    def on_enter_seek(self):
        print('Making a PI/8 rad turn.')
        Global.cmd = (i2c.CMD_TURN,self.SEEK_ANGLE)
        if Global.isMarker == True:
            self.next = self.enter
        else:
            self.next = self.noMarker

    def check_marker(self):
        if Global.isMarker == False:
            if self.take == 2:
                self.take = 0
                return False
            else:
                self.take += 1
                Global.cmd = (i2c.CMD_TURN,0)
                return True
        else:
            self.take = 0
            return True
        

    def on_enter_correct(self):
        print('Centering Aruco Marker in camera\'s view.')
        Global.cmd = (i2c.CMD_TURN,Global.camAngle)
        if Global.isCentered == True:
            self.next = self.center
        elif not self.check_marker():
            self.next = self.exit
        else:
            self.next = self.retryCenter

    def on_enter_forward(self):
        print('Moving towards Aruco Marker.')
        Global.cmd = (i2c.CMD_VELOC,self.FWD_SPEED)
        if Global.isExitBottom == True:
            self.next = self.bottom
        elif not self.check_marker():
            self.next = self.whereDidHeGo
        elif Global.isCentered == False:
            self.next = self.side
        else:
            self.next = self.onward

    def on_enter_moveOver(self):
        print('Moving over Aruco Marker.')
        Global.cmd = (i2c.CMD_POS,self.DEAD_RECON_DISTANCE)
        if Global.arucoIndex >= Global.maxArucoIndex:
            self.next = self.end
        else:
            Global.arucoIndex += 1
            self.next = self.cont

    def on_enter_stop(self):
        print('Haulting rover.')
        Global.cmd = (0xFF,0x04)
