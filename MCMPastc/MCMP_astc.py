from MCMPinstance import  MCMPInstance
from math import  ceil
from collections import  namedtuple
from enum import  Enum
import networkx as nx
import matplotlib.pyplot as plt
import sympy
from  drawEnv import drawPic,drawSTCPic

class STCVertType(Enum):
    wayVert = 0
    single = 1
    doubleSame = 2
    triple = 3
    obstacle = 4
    doubleDiff = 5
    init = 6

class STCDir(Enum):
    left = 0
    right = 1
    top = 2
    bottom = 3

class STCVirtualVertType(Enum):
    NoVir = 0
    VLB = 1
    VRT = 2
    VLT = 3
    VRB = 4



GridInd = namedtuple('GridInd', ['row', 'col'])
STCGridInd = namedtuple('STCGridInd',['row', 'col','virType'])



class STCVert(object):
    def __init__(self, i, j):
        self._gridInd = GridInd(i, j)
        self._pos_x = 2 * i + 1
        self._pos_y = 2 * j + 1
        self._LBInd = GridInd(2 * i , 2 * j)
        self._LTInd = GridInd(2 * i , 2 * j + 1)
        self._RBInd = GridInd(2 * i + 1, 2 * j)
        self._RTInd = GridInd(2 * i + 1, 2 * j + 1)
        self._type = STCVertType.init

    def __str__(self):
        return 'x = ' + str(self._pos_x) + ' y = '+ str(self._pos_y)


class STC_Map(object):
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

        self._s_row = ceil(self._row/2)
        self._s_col = ceil(self._col/2)

        self._setSTC = set()
        self._indSet = set()
        self._vitualIndSet = set()
        self._stcVitualIndSet = set()
        self._stcGraph = nx.Graph()
        self.generateGraph()

    def generateGraph(self):

        # print(self._s_row)
        for i in range(self._s_row):
            for j in range(self._s_col):
                vert = STCVert(i, j)
                self._setSTC.add(GridInd(i, j))
                self._indSet.add(GridInd(i, j))
                gridIndLst = self.getTGridBySTC(GridInd(i, j ))
                obNum = self.countObNum(gridIndLst)
                virType = STCVirtualVertType.NoVir
                if obNum == 2:
                    vert._type = STCVertType.doubleSame
                    if self._mat[vert._LTInd.row][vert._LTInd.col] == 1 and self._mat[vert._RBInd.row][vert._RBInd.col] == 1:
                        vert._type = STCVertType.doubleDiff
                        stc_vertInd = STCGridInd(row= i, col = j, virType = STCVirtualVertType.VRT)
                        vert._pos_x += 0.1
                        vert._pos_y += 0.1
                        self._stcGraph.add_node(stc_vertInd,vert = vert)
                        self._stcVitualIndSet.add(stc_vertInd)
                        stc_vertInd = STCGridInd(row= i, col = j, virType = STCVirtualVertType.VLB)
                        vert = STCVert(i,j)
                        vert._type =STCVertType.doubleDiff
                        vert._pos_x -= 0.1
                        vert._pos_y -= 0.1
                        self._stcGraph.add_node(stc_vertInd,vert = vert)
                        self._stcVitualIndSet.add(stc_vertInd)
                        continue
                    if self._mat[vert._RTInd.row][vert._RTInd.col] == 1 and self._mat[vert._LBInd.row][vert._LBInd.col] == 1:
                        vert._type = STCVertType.doubleDiff
                        stc_vertInd = STCGridInd(row= i, col = j, virType = STCVirtualVertType.VLT)
                        vert._pos_x -= 0.1
                        vert._pos_y += 0.1
                        self._stcGraph.add_node(stc_vertInd,vert = vert)
                        self._stcVitualIndSet.add(stc_vertInd)
                        vert = STCVert(i,j)
                        vert._type =STCVertType.doubleDiff
                        vert._pos_x += 0.1
                        vert._pos_y -= 0.1
                        stc_vertInd = STCGridInd(row= i, col = j, virType = STCVirtualVertType.VRB)
                        self._stcGraph.add_node(stc_vertInd,vert = vert)
                        self._stcVitualIndSet.add(stc_vertInd)
                        continue
                if obNum == 0 :
                    vert._type = STCVertType.wayVert
                if obNum == 1:
                    vert._type = STCVertType.single
                if obNum == 3:
                    vert._type = STCVertType.triple
                if obNum == 4:
                    vert._type = STCVertType.obstacle
                stc_vertInd = STCGridInd(row= i, col = j, virType = STCVirtualVertType.NoVir)
                self._stcGraph.add_node(stc_vertInd, vert=vert)
                pass

        for graphInd in self._stcGraph:
            print(self._stcGraph.nodes[graphInd]['vert'])
            vert = self._stcGraph.nodes[graphInd]['vert']
            if vert._type != STCVertType.obstacle:
                neiLst = self.getSTCNeighbor(graphInd)
                print(neiLst)
                # raise Exception('xx')
                for neiInd in neiLst:
                    self._stcGraph.add_edge(graphInd, neiInd)

        print(self._stcGraph.number_of_edges())
        # print(graphInd)
        # raise  Exception('xx')

        # nx.draw(self._stcGraph)
        # print(self._stcGraph)

    def getTGridBySTC(self,cenInd :GridInd):
        maxRow = cenInd.row * 2
        maxCol = cenInd.col * 2
        resLst = []

        if (maxRow == self._row) and (maxCol == self._col):
            resLst.append(GridInd(cenInd.row * 2, cenInd.col * 2))
            return resLst
        if maxRow == self._row:
            resLst.append(GridInd(cenInd.row * 2, cenInd.col * 2))
            resLst.append(GridInd(cenInd.row * 2, cenInd.col * 2 + 1))
            return resLst
        if maxCol == self._col:
            resLst.append(GridInd(cenInd.row * 2, cenInd.col * 2))
            resLst.append(GridInd(cenInd.row * 2 + 1, cenInd.col * 2))
            return resLst
        resLst.append(GridInd(cenInd.row * 2, cenInd.col * 2))
        resLst.append(GridInd(cenInd.row * 2, cenInd.col * 2 + 1))
        resLst.append(GridInd(cenInd.row * 2 + 1, cenInd.col * 2))
        resLst.append(GridInd(cenInd.row * 2 + 1, cenInd.col * 2 + 1))
        return resLst

    def getSTCNeighbor(self,stc_gridInd: STCGridInd):
        res = []
        ad_left = False
        ad_right = False
        ad_top = False
        ad_bottom = False
        if stc_gridInd.virType == STCVirtualVertType.NoVir:
            ad_left = True
            ad_right = True
            ad_top = True
            ad_bottom = True

        if stc_gridInd.virType == STCVirtualVertType.VLB:
            ad_left = True
            ad_bottom = True
        if stc_gridInd.virType == STCVirtualVertType.VRT:
            ad_right = True
            ad_top = True
        if stc_gridInd.virType == STCVirtualVertType.VLT:
            ad_left = True
            ad_top = True
        if stc_gridInd.virType == STCVirtualVertType.VRB:
            ad_right = True
            ad_bottom = True

        if ad_left:
            grid_ind = GridInd(row = stc_gridInd.row - 1, col = stc_gridInd.col)
            if grid_ind in self._indSet:
                virType = STCVirtualVertType.NoVir
                if grid_ind in self._vitualIndSet:
                    if STCGridInd(row = grid_ind.row, col = grid_ind.col, virType= STCVirtualVertType.VRT) in self._stcVitualIndSet:
                        virType = STCVirtualVertType.VRT
                    else:
                        virType = STCVirtualVertType.VRB
                nei_stcGridInd = STCGridInd(stc_gridInd.row - 1, stc_gridInd.col, virType)
                if self.adjacent(stc_gridInd, nei_stcGridInd, STCDir.left):
                    res.append(nei_stcGridInd)
        if ad_right:
            grid_ind = GridInd(row = stc_gridInd.row + 1, col = stc_gridInd.col)
            if grid_ind in self._indSet:
                virType = STCVirtualVertType.NoVir
                if grid_ind in self._vitualIndSet:
                    if STCGridInd(row = grid_ind.row, col = grid_ind.col, virType= STCVirtualVertType.VLT) in self._stcVitualIndSet:
                        virType = STCVirtualVertType.VLT
                    else:
                        virType = STCVirtualVertType.VRB
                nei_stcGridInd = STCGridInd(stc_gridInd.row + 1, stc_gridInd.col, virType)
                if self.adjacent(stc_gridInd, nei_stcGridInd, STCDir.left):
                    res.append(nei_stcGridInd)
        if ad_top:
            grid_ind = GridInd(row = stc_gridInd.row, col = stc_gridInd.col + 1)
            if grid_ind in self._indSet:
                virType = STCVirtualVertType.NoVir
                if grid_ind in self._vitualIndSet:
                    if STCGridInd(row = grid_ind.row, col = grid_ind.col, virType= STCVirtualVertType.VRB) in self._stcVitualIndSet:
                        virType = STCVirtualVertType.VRB
                    else:
                        virType = STCVirtualVertType.VLB
                nei_stcGridInd = STCGridInd(stc_gridInd.row, stc_gridInd.col + 1, virType)
                if self.adjacent(stc_gridInd, nei_stcGridInd, STCDir.left):
                    res.append(nei_stcGridInd)
        if ad_bottom:
            grid_ind = GridInd(row = stc_gridInd.row, col = stc_gridInd.col - 1)
            if grid_ind in self._indSet:
                virType = STCVirtualVertType.NoVir
                if grid_ind in self._vitualIndSet:
                    if STCGridInd(row = grid_ind.row, col = grid_ind.col, virType= STCVirtualVertType.VRT) in self._stcVitualIndSet:
                        virType = STCVirtualVertType.VRT
                    else:
                        virType = STCVirtualVertType.VLT
                nei_stcGridInd = STCGridInd(stc_gridInd.row, stc_gridInd.col - 1, virType)
                if self.adjacent(stc_gridInd, nei_stcGridInd, STCDir.left):
                    res.append(nei_stcGridInd)

        return res
        pass


    def adjacent(self, sInd:STCGridInd, tInd:STCGridInd, dir ):
        if sInd not in self._stcGraph:
            print(sInd)
            # raise  Exception('adjacent error')
            return False
        if tInd not in self._stcGraph:
            print('tInd = ', tInd)
            print(dir)
            # raise  Exception('adjacent error')
            return False
        sVert = self._stcGraph.nodes[sInd]['vert']
        tVert = self._stcGraph.nodes[tInd]['vert']

        gridInd_row = int((sVert._pos_x + tVert._pos_x)/2 - 1)
        gridInd_col = int((sVert._pos_y + tVert._pos_y)/2 - 1)
        '''
        此处存在问题
        '''
        obGridLst = []

        print('row = ',gridInd_row)
        print('col = ',gridInd_col)
        print('sVert._pos_x',sVert._pos_x)
        print('tVert._pos_x',tVert._pos_x)
        print('sVert._pos_y',sVert._pos_y)
        print('tVert._pos_y',tVert._pos_y)
        print('sInd = ', sInd)
        print('tInd = ', tInd)
        if sInd == STCGridInd(5,0,STCVirtualVertType.VLT) and tInd == STCGridInd(5,1,STCVirtualVertType.NoVir):
            print('xxx here')
            # raise Exception('XX')

        if self._mat[gridInd_row][gridInd_col] == 1:
            obGridLst.append(GridInd(gridInd_row, gridInd_col))
        if self._mat[gridInd_row + 1][gridInd_col] == 1:
            obGridLst.append(GridInd(gridInd_row + 1, gridInd_col))
        if self._mat[gridInd_row][gridInd_col + 1] == 1:
            obGridLst.append(GridInd(gridInd_row, gridInd_col + 1))
        if self._mat[gridInd_row + 1][gridInd_col + 1] == 1:
            obGridLst.append(GridInd(gridInd_row + 1, gridInd_col + 1))

        if len(obGridLst) <= 1:
            return True
        if len(obGridLst) >= 3:
            return False
        # len)(obGridLst == 2
        line1 = sympy.Line2D(sympy.Point(sInd.row * 2, sInd.col * 2), sympy.Point(tInd.row * 2, tInd.col * 2))
        line2 = sympy.Line2D(sympy.Point(obGridLst[0].row, obGridLst[0].col), sympy.Point(obGridLst[1].row, obGridLst[1].col))

        if sympy.Line2D.is_parallel(line1,line2):
            return True
        else:
            # print(line1,line2)
            # raise Exception('xx')
            return False


        # if dir == STCDir.left:
        #     if not self.obstacleOccupyDir(sInd, STCDir.left) and not self.obstacleOccupyDir(tInd,STCDir.right):





    def countObNum(self, gridIndLst):
        obNum = 0
        # print(gridIndLst)
        for gridInd in gridIndLst:
            if self._mat[gridInd.row][gridInd.col] == 1:
                obNum = obNum + 1
        return obNum




    def obstacleOccupyDir(self, gridInd: STCGridInd, dir):
        if gridInd not in self._setSTC:
            raise Exception('obstacleOccupyDir')
            return False
        vert = self._stcGraph.nodes[gridInd]['vert']
        if dir == STCDir.left:
            if self.gridObstacle(vert._LBInd) and self.gridObstacle(vert._LTInd):
                return True
            else:
                return False
        if dir == STCDir.right:
            if self.gridObstacle(vert._RBInd) and self.gridObstacle(vert._RTInd):
                return True
            else:
                return False
        if dir == STCDir.top:
            if self.gridObstacle(vert._LTInd) and self.gridObstacle(vert._RTInd):
                return True
            else:
                return False
        if dir == STCDir.bottom:
            if self.gridObstacle(vert._LBInd) and self.gridObstacle(vert._RBInd):
                return True
            else:
                return False

            pass

    def gridObstacle(self,gridInd: GridInd):
        if self._mat[gridInd.row][gridInd.col] == 1:
            return True
        else:
            return False
    def __str__(self):
        return "stc_map _s_row = " + str(self._s_row)  +' _s_col = ' + str(self._s_col)




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
        self._s_map = STC_Map(self._ins)

        print(self._s_map)
        pass

    def formSpanningTree(self):
        pass

    def __str__(self):
        return 'mcmp_astc row = '  + str(self._row) + ' col = ' + str(self._col)

if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    mcmp_astc = MCMP_ASTC(ins)
    mcmp_astc.plan()
    print(mcmp_astc)
    print('xx')
    # plt.show()
    # print(mcmp_astc._s_map._stcGraph.edges())
    edgeLst = []
    _graph = mcmp_astc._s_map._stcGraph
    for edge in mcmp_astc._s_map._stcGraph.edges():
        # print(edge)
        sPnt_x = _graph.nodes[edge[0]]['vert']._pos_x
        sPnt_y = _graph.nodes[edge[0]]['vert']._pos_y
        tPnt_x = _graph.nodes[edge[1]]['vert']._pos_x
        tPnt_y = _graph.nodes[edge[1]]['vert']._pos_y
        edgeLst.append((sPnt_x, sPnt_y, tPnt_x, tPnt_y))

        # raise Exception('xx')
    drawSTCPic(ins, edgePntLst = edgeLst)




