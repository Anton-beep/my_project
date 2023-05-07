MOTORS_MAX_POWER = 100
SAFE_START_ACCEL = 0.5
SAFE_END_ACCEL = 0.5


def tweaker(dist, startPowB, startPowC, endPowB, endPowC, startAccel, endAccel):
    if startPowB > startPowC:
        startAccelB = startAccel
        startAccelC = startAccel * startPowC / startPowB
        endAccelB = endAccel
        endAccelC = endAccel * startPowC / startPowB
        startAccelMaxDegB = (pow(MOTORS_MAX_POWER, 2) -
                             pow(startPowB, 2)) / (2 * startAccelB)
        startAccelMaxDegC = startAccelMaxDegB * startPowC / startPowB
        endAccelMaxDegB = (pow(MOTORS_MAX_POWER, 2) -
                           pow(endPowB, 2)) / (2 * endAccelB)
        endAccelMaxDegC = startAccelMaxDegB * startPowC / startPowB
        maxPowerB = MOTORS_MAX_POWER
        maxPowerC = maxPowerB * startPowB / startPowC
    elif startPowC < startPowB:
        startAccelB = startAccel * startPowB / startPowC
        startAccelC = startAccel
        endAccelB = endAccel
        endAccelC = endAccel * startPowC / startPowB
        startAccelMaxDegC = (pow(MOTORS_MAX_POWER, 2) -
                             pow(startPowC, 2)) / (2 * startAccelC)
        startAccelMaxDegB = startAccelMaxDegC * startPowB / startPowC
        endAccelMaxDegC = (pow(MOTORS_MAX_POWER, 2) -
                           pow(endPowC, 2)) / (2 * endAccelC)
        endAccelMaxDegB = startAccelMaxDegB * startPowB / startPowC
        maxPowerC = MOTORS_MAX_POWER
        maxPowerB = maxPowerC * startPowC / startPowB
    else:
        startAccelB = startAccel
        startAccelC = startAccel
        endAccelB = endAccel
        endAccelC = endAccel * startPowC / startPowB
        startAccelMaxDegB = (pow(MOTORS_MAX_POWER, 2) -
                             pow(startPowB, 2)) / (2 * startAccelB)
        startAccelMaxDegC = startAccelMaxDegB * startPowC / startPowB
        endAccelMaxDegB = (pow(MOTORS_MAX_POWER, 2) -
                           pow(endPowB, 2)) / (2 * endAccelB)
        endAccelMaxDegC = startAccelMaxDegB * startPowC / startPowB
        maxPowerB = MOTORS_MAX_POWER
        maxPowerC = maxPowerB * startPowB / startPowC

    if startAccelMaxDegB + endAccelMaxDegB > dist:
        return f'moveBCCustomAccelMainB({startAccelMaxDegB}, {startPowB}, {startPowC}, {startAccelB}, {startAccelC})\n' \
               f'moveBC({dist - startAccelMaxDegB - startAccelMaxDegC})\n' \
               f'moveBCCustomAccelMainB({endAccelMaxDegB}, {maxPowerB}, {maxPowerC}, {endAccelB}, {endAccelC})'

print(tweaker(300, -20, 20, -18, 18, 1, 1))