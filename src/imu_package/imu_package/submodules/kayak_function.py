#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

class KayakFunctions:
    def __init__(self):
        # Variable to save wether the kayak is rencentring 
        self.recentring = True

    """
    Return the intensity the kayayer must turn between -1 and 1.
    -1 corresponds to a need to rotate right and 1 to rotate left
    @param angle: The angle of the kayak in rad
    @param position: Horizontal position of the kayak in meter
    @param max_angle: Maximum rotation authorised for the kayak
    @param max_dist: Maximum distance forward targeted by the kayak
    @param min_dist: Closest ditance targeted by the kayak
    @return Intensity of the bip between -1 and 1
    """
    def getOrder(self, angle: float, position: float, max_angle: float, max_dist: float, min_dist: float) -> float:
        abs_angle = abs(angle)
        signe_angle = 1 if angle > 0 else -1
        
        abs_position = abs(position)
        signe_position = 1 if position > 0 else -1

        if abs_position < 0.25: # If the kayal is in the center
            five_degree_rad = 0.087266
            # If the kayak aims outside
            if (signe_angle * signe_position < 0 and abs_angle != 0) or abs_angle > max_angle:
                bip = signe_angle * min(1., abs_angle / five_degree_rad)

            else:
                bip = 0.

        elif abs_angle != 0: # If the kayak doesn't point exactly forward
            opposed_side = math.tan(math.pi / 2 - abs_angle) * abs_position # Calculate the length of the opposed side

            if not self.recentring:
                self.recentring = opposed_side > (max_dist + min_dist) / 2
                bip = signe_position * max(-1., min(1., opposed_side / ((max_dist + min_dist) / 2) - 1))

            elif signe_angle * signe_position < 0 and abs_angle: # If the kayak aims outside
                bip = signe_position

            elif max_dist < opposed_side: # If the kayak aims too far away
                bip = -signe_position * max(-1., min(1., opposed_side / max_dist - 1))

            elif min_dist > opposed_side or not self.recentring: # If the kayak aims too closely
                bip = signe_position * max(-1., min(1., opposed_side / ((max_dist + min_dist) / 2) - 1))
                self.recentring = False

            else: 
                bip = 0.

        else:
            bip = 0.
        
        return bip

if __name__ == '__main__':
    kayak = KayakFunctions()
    for i in range(18):
        print(kayak.getOrder(i * 0.01, 1., 5., 20., 10.))
