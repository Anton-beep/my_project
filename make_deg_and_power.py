RAT_INC = 0.2
RAT_KEEP = 0.55
RAT_DEC = 0.25


def count_ratio(deg):
    global RAT_INC
    global RAT_KEEP
    global RAT_DEC
    return deg * RAT_INC, deg * RAT_KEEP, deg * RAT_DEC

while True:
    print(count_ratio(int(input())))
