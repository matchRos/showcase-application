#!/usr/bin/env python

class ButtonDetectionClass():
    def __init__(self):
        self._buttons_pressed = {}

    def _is_one_button_pressed(self):
        for button_pressed in self._buttons_pressed.itervalues():
            if button_pressed:
                return button_pressed
        
        return False