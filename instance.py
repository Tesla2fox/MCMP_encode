

import random
import numpy as np

class MCMPInstance(object):
    def __init__(self,row,col,obstacleLst,robPosLst):
        self._row = row
        self._col = col
        self._obstacleLst  = obstacleLst
        self._mat = np.zeros([self._row,self._col])
        # print(self._mat)
        ind = 0
        for rowInd in range(row):
            for colInd in range(col):
                self._mat[rowInd][colInd] = self._obstacleLst[ind]
                ind  = ind + 1
        print(self._mat)
        self._robPosLst = robPosLst



if __name__ =='__main__':
    row = 20
    col = 20
    robNum = 2
    p = np.array([0.9,0.1])
    np.random.seed(1000)
    rob_x = np.random.randint(20,size = robNum)
    rob_y = np.random.randint(20,size = robNum)
    robPosLst = []
    for i in range(robNum):
        robPosLst.append((rob_x[i],rob_y[i]))
    # 0 means obstacles
    # 1 means way
    print(robPosLst)
    obstacleLst = []
    for rowInd in range(row):
        for colInd in range(col):
            if (rowInd,colInd) in robPosLst:
                obstacleLst.append(0)
            else:
                obstacleLst.append(np.random.choice([0,1],p =p.ravel()))
    ins = MCMPInstance(row,col,obstacleLst,robPosLst)



    print(obstacleLst)
