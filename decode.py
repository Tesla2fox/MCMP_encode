
import numpy as np
import instance







class MCMPDecode(object):
    def __init__(self,ins : instance.MCMPInstance):
        self._pathLst = []
        self._ins = ins
        self._mat = ins._mat
        self._robPosLst =

        pass
    def MCMPDecode(self,x):
        pass





if __name__ == '__main__':
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
    # 1 means obstacles
    # 0 means way
    print(robPosLst)
    obstacleLst = []
    for rowInd in range(row):
        for colInd in range(col):
            if (rowInd,colInd) in robPosLst:
                obstacleLst.append(0)
            else:
                obstacleLst.append(np.random.choice([0,1],p =p.ravel()))
    ins =    instance.MCMPInstance(row,col,obstacleLst,robPosLst)