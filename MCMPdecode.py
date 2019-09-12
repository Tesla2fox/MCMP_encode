import numpy
import drawEnv
from MCMPastar import MCMP_Solver
from MCMPinstance import MCMPInstance



class MCMP_Decode(object):
    def __init__(self,ins: MCMPInstance):
        self._ins = ins
        self._row = ins._row
        self._col = ins._col
        self._ins = ins
        self._mat = ins._mat
        self._robNum = ins._robNum
        self._robPosLst = ins._robPosLst
        self._robRowLst = ins._robRowLst
        self._robColLst = ins._robColLst
        self._robReachRowLst = ins._robReachRowLst
        self._robReachColLst = ins._robReachColLst

        self._mcmp_astar = MCMP_Solver(ins)

        print(self._mcmp_astar)

    def __str__(self):
        return str(self._ins)

if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('.\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    decode = MCMP_Decode(ins)
    print(decode)
