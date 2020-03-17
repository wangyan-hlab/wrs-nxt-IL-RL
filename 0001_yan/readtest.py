import os
import utiltools.robotmath as rm
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import numpy as np
def as_num(x):
    y = '{:.10f}'.format(x)  # .10f 保留10位小数
    return y
def readtest(filename,start,end):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", filename), "r")
    f1 = f.readlines()[start:end]
    means = []
    for string in f1:
        strlist = string.strip(" ").strip("[").replace("]", "").rstrip(" ")
        s = strlist.split()
        print(s)
        for ss in s:
            if 'E' in ss or 'e' in ss:
                x = as_num(float(ss))
                print(x)
                means.append(float(x))
            else:
                print(float(ss))
                means.append(float(ss))
    return means

if __name__ == '__main__':
    C = readtest("GMM_parameters_1.txt",156,160)
    print(C)
