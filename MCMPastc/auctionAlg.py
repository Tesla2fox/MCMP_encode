import stcMap.stcMap
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd
from MCMPinstance import  MCMPInstance



class AuctionAlgSTC(object):
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
        self._s_map = STC_Map(self._ins)

        self._robEstCostLst = [0 for x in range(self._robNum)]
        self._robSetLst = [set() for x in range(self._robNum)]
        self._robNeiSetLst = [set() for x in range(self._robNum)]
        

    def auction(self):
        for robID in range(self._robNum):
            ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))


    def plan(self):
        print(self._s_map)
        pass

    def formSpanningTree(self):
        pass

    def __str__(self):
        return 'mcmp_astc row = ' + str(self._row) + ' col = ' + str(self._col)


if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\pycode\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    astc = AuctionAlgSTC(ins)


    print('xx')