#the  instance generator


import numpy as np
import networkx as nx
import readcfg as rd
import datetime
import MCMPinstance
import drawEnv as draw


def getNeighbor(envMat,lst = (0,0),row =20, col =20):
    # print(lst)
    resLst = []
    #left
    lstLeft = (lst[0]-1,lst[1])
    if(lstLeft[0]>=0):
        if(envMat[lstLeft[0]][lstLeft[1]]== 0):
            resLst.append(lstLeft)
    #right
    lstRight = (lst[0]+1,lst[1])
    if(lstRight[0]<row):
        if(envMat[lstRight[0]][lstRight[1]]== 0):
            resLst.append(lstRight)
    #top
    lstTop = (lst[0],lst[1]+1)
    if(lstTop[1]<col):
        if(envMat[lstTop[0]][lstTop[1]]== 0):
            resLst.append(lstTop)
    #bottom
    lstBottom = (lst[0],lst[1]-1)
    if(lstBottom[1]>=0):
        if(envMat[lstBottom[0]][lstBottom[1]]== 0):
            resLst.append(lstBottom)
    return resLst


if __name__ == '__main__':
    print('the generator')
    for i in range(10):
        row = 50
        col = 50
        robNum = 2 + i
        p = np.array([0.7,0.3])
        r_seed = 2
        np.random.seed(r_seed)

        rob_row_lst = np.random.randint(row,size = robNum)
        rob_col_lst = np.random.randint(col,size = robNum)

        robPosLst = []
        for i in range(robNum):
            robPosLst.append((rob_row_lst[i],rob_col_lst[i]))

        # 0 means way
        # 1 means obstacles
        print(robPosLst)
        obstacleLst = []
        for rowInd in range(row):
            for colInd in range(col):
                if (rowInd,colInd) in robPosLst:
                    obstacleLst.append(0)
                else:
                    obstacleLst.append(np.random.choice([0,1],p =p.ravel()))

        _mat = np.zeros([row,col])

        ind = 0
        for rowInd in range(row):
            for colInd in range(col):
                _mat[rowInd][colInd] = obstacleLst[ind]
                ind = ind + 1

        # print(_mat)

        edgeLst = []
        sPntx = []
        sPnty = []
        tPntx = []
        tPnty = []
        G = nx.Graph()
        for i in range(row):
            for j in range(col):
                centre = (i, j)
                G.add_node((i, j))
                if (_mat[i][j] == 1):
                    continue
                neiLst = getNeighbor(_mat, lst=centre, row=row, col=col)
                # print(neiLst)
                # raise Exception()
                for unit in neiLst:
                    sPntx.append(i + 0.5)
                    sPnty.append(j + 0.5)
                    tPntx.append(unit[0] + 0.5)
                    tPnty.append(unit[1] + 0.5)
                    G.add_edge(centre, unit)
                    if (i, j, unit[0], unit[1]) not in edgeLst  or (unit[0], unit[1], i, j) not in edgeLst:
                        edgeLst.append((i, j, unit[0], unit[1]))

        # print(len(nx.connected_components(G)))

        component = list(nx.connected_components(G))

        print('component = ', component)
        # print(len(component))

        # exit()
        reachComponentLst = []
        unReachCompLst = [n for n in range(len(component))]
        # print(unReachCompLst)

        for i in range(robNum):
            for j in range(len(component)):
                if (reachComponentLst.count(j) != 1):
                    if ((rob_row_lst[i], rob_col_lst[i]) in component[j]):
                        reachComponentLst.append(j)
                        unReachCompLst.remove(j)



        _robReachRowLst = []
        _robReachColLst = []
        for unit in reachComponentLst:
            for gridUnit in component[unit]:
                _robReachRowLst.append(gridUnit[0])
                _robReachColLst.append(gridUnit[1])

        print(_robReachRowLst)
        print(_robReachColLst)

        _robUnReachRowLst = []
        _robUnReachColLst = []

        for unit in unReachCompLst:
            for gridUnit in component[unit]:
                _robUnReachRowLst.append(gridUnit[0])
                _robUnReachColLst.append(gridUnit[1])


        fileCfg = './/benchmark//r'+ str(robNum)+'_r'+str(row)+'_c'+str(col)+'_p'+str(p[0])+'_s'+str(r_seed)+'_Outdoor_Cfg.dat'
        f_con = open(fileCfg,'w')

        f_con.write('time '+ datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+'\n')
        rd.writeConf(f_con, 'row', [row])
        rd.writeConf(f_con, 'col', [col])
        rd.writeConf(f_con, 'robRow', rob_row_lst)
        rd.writeConf(f_con, 'robCol', rob_col_lst)
        # rd.writeConf(f_con, 'obstacle', obstacleLst)

        rd.writeConf(f_con,'robReachRowLst',_robReachRowLst)
        rd.writeConf(f_con,'robReachColLst',_robReachColLst)
        rd.writeConf(f_con, 'robUnReachRowLst', _robUnReachRowLst)
        rd.writeConf(f_con, 'robUnReachColLst', _robUnReachColLst)

        grid = []
        for rowID,colID in np.ndindex(_mat.shape):
            grid.append(int(_mat[rowID][colID]))
        rd.writeConf(f_con,'grid',grid)


        #
        # for rowInd in range(row):
        #     for colInd in range(col):
        #
        #         if (rowInd,colInd) in robPosLst:
        #             obstacleLst.append(0)
        #         else:
        #             obstacleLst.append(np.random.choice([0,1],p =p.ravel()))

        f_con.close()


    '''
    generator benchmark over
    '''

    ins = MCMPinstance.MCMPInstance()
    ins.loadCfg(fileCfg)
    # ins.setPara(row, col, obstacleLst, robPosLst)
    draw.drawPic(ins)
    # print('edgeLst = ', edgeLst)
    # draw.drawPic(ins, edgeLst = edgeLst)

