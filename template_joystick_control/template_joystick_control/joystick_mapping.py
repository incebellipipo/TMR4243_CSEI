
LEFT_TRIGGER=0
RIGHT_TRIGGER=0
LEFT_STICK_VERTICAL=0
LEFT_STICK_HORIZONTAL=0
RIGHT_STICK_VERTICAL=0
RIGHT_STICK_HORIZONTAL=0

A_BUTTON=0 # A button for Xbox, (Cross)     x button for PS
B_BUTTON=0 # B button for Xbox, (Square)    ⬜ button for PS
X_BUTTON=0 # X button for Xbox, (Circle)    ◯ button for PS
Y_BUTTON=0 # Y button for Xbox, (Triangle)  △ button for PS

import dataclasses
class JoystickMapping(dataclasses.dataclass):
    LEFT_STICK_HORIZONTAL: int
    LEFT_STICK_VERTICAL: int
    RIGHT_STICK_HORIZONTAL: int
    RIGHT_STICK_VERTICAL: int
    LEFT_TRIGGER: int
    RIGHT_TRIGGER: int
    A_BUTTON: int
    B_BUTTON: int
    X_BUTTON: int
    Y_BUTTON: int