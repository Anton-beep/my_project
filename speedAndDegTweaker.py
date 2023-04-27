"""Tweaker for "3parts" moving function with 3 accelerations"""
import click
import numpy as np

#
# раскаидавть расстояния по отношению ускорений и считать макс скорость через полученное расстояние
#

# speeds and accels
SAFE_START_ACCEL = 10  # deg * s ** -2
SAFE_END_ACCEL = 5  # deg * s ** -2
SAFE_START_SPEED = 10  # deg * s ** -1
SAFE_MAX_SPEED = 95  # deg * s ** -1
SAFE_END_SPEED = 17  # deg * s ** -1

SAFE_INC_DEG = np.int64(SAFE_MAX_SPEED ** 2 -
                        SAFE_START_SPEED ** 2) / np.int64(2 * SAFE_START_ACCEL)
SAFE_DEC_DEG = np.int64(SAFE_MAX_SPEED ** 2 -
                        SAFE_END_SPEED ** 2) / np.int64(2 * SAFE_END_ACCEL)

EXTREME_START_ACCEL = 25  # deg * s ** -2
EXTREME_END_ACCEL = 25  # deg * s ** -2
EXTREME_START_SPEED = 23  # deg * s ** -1
EXTREME_MAX_SPEED = 100  # deg * s ** -1
EXTREME_END_SPEED = 22  # deg * s ** -1

EXTREME_INC_DEG = np.int64(
    EXTREME_MAX_SPEED ** 2 - EXTREME_START_SPEED ** 2) / np.int64(2 * EXTREME_START_ACCEL)
EXTREME_DEC_DEG = np.int64(
    EXTREME_MAX_SPEED ** 2 - EXTREME_END_SPEED ** 2) / np.int64(2 * EXTREME_END_ACCEL)


def take_float_part(num):
    return np.float64(num - np.int64(num))


@click.command()
@click.option('-s', 'safe', is_flag=True, default=True, help='safe mode')
@click.option('-e', 'extreme', is_flag=True, default=False, help='extreme mode')
def tweaker(safe, extreme):
    deg = click.prompt('deg?')
    if deg == '':
        quit()
    deg = int(deg)

    if extreme:
        inc_deg = np.int64(EXTREME_INC_DEG)
        dec_deg = np.int64(EXTREME_DEC_DEG)
        start_speed = np.int64(EXTREME_START_SPEED)
        max_speed = np.int64(EXTREME_MAX_SPEED)
        end_speed = np.int64(EXTREME_END_SPEED)
        start_accel = np.float64(EXTREME_START_ACCEL)
        end_accel = np.float64(EXTREME_END_ACCEL)
    else:
        inc_deg = np.int64(SAFE_INC_DEG)
        dec_deg = np.int64(SAFE_DEC_DEG)
        start_speed = np.int64(SAFE_START_SPEED)
        max_speed = np.int64(SAFE_MAX_SPEED)
        end_speed = np.int64(SAFE_END_SPEED)
        start_accel = np.float64(SAFE_START_ACCEL)
        end_accel = np.float64(SAFE_END_ACCEL)

    if deg > inc_deg + dec_deg:
        """real 3 parts"""
        max_deg = deg - inc_deg - dec_deg
        dec_deg = np.round(
            dec_deg + take_float_part(inc_deg) + take_float_part(max_deg))
        click.echo(
            f'({np.int64(inc_deg)}, {np.int64(max_deg)}, {np.int64(dec_deg)}, -{start_speed}, {start_speed}, -{max_speed}, {max_speed}, -{end_speed}, {end_speed})')
    else:
        """only 2 parts"""
        dec_deg = deg / (start_accel / end_accel + 1)
        inc_deg = deg - dec_deg
        dec_deg = np.round(dec_deg + take_float_part(inc_deg))
        max_deg = 0
        max_speed = np.sqrt(2 * end_accel * dec_deg + start_speed ** 2)
        click.echo(
            f'({np.int64(inc_deg)}, {np.int64(max_deg)}, {np.int64(dec_deg)}, -{start_speed}, {start_speed}, -{np.round(max_speed)}, {np.round(max_speed)}, -{end_speed}, {end_speed})')


if __name__ == '__main__':
    tweaker()
