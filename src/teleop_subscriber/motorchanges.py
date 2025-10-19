from msg import MotorChanges, MotorCurrent, StickPosition
from .control import WheelSpeeds
def motorchanges(wheelspeeds: WheelSpeeds) -> MotorChanges:
    LeftWheels = round(wheelspeeds.left*127)+127
    RightWheels = round(wheelspeeds.right*127)+127
    return MotorChanges(Changes=[MotorCurrent(index=MotorCurrent.FRONTRIGHT,MotorVelocity=RightWheels),
                                 MotorCurrent(index=MotorCurrent.FRONTLEFT,MotorVelocity=LeftWheels),
                                 MotorCurrent(index=MotorCurrent.BACKRIGHT,MotorVelocity=RightWheels),
                                 MotorCurrent(index=MotorCurrent.BACKLEFT,MotorVelocity=LeftWheels)])

