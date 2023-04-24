import numpy as np

FILE_TO_READ = r"C:\Users\anton\Downloads\calibratedHSV.txt"

def __main__():
    with open(FILE_TO_READ, "r", encoding="utf-8") as f:
        data = f.read()

    HVals = list()
    SVals = list()
    VVals = list()

    for el in data.split("!")[:-1]:
        el = el.replace("\x00", "")
        HVals.append(float(el.split(";")[0]))
        SVals.append(float(el.split(";")[1]))
        VVals.append(float(el.split(";")[2]))

    print("Hue: ", np.mean(HVals))
    print("Saturation: ", np.mean(SVals))
    print("Value: ", np.mean(VVals))



if __name__ == "__main__":
    __main__()
