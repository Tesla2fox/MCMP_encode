from MCMPinstance import  MCMPInstance

class MCMP_ASTC(object):
    def __init__(self,ins:MCMPInstance):

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


        pass

    def plan(self):
        pass

    def

    def __str__(self):
        return 'mcmp_astc row = '  + str(self._row) + ' col = ' + str(self._col)

if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    mcmp_astc = MCMP_ASTC(ins)
    print(mcmp_astc)
    print('xx')



