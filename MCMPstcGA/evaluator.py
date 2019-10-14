from MCMPinstance import MCMPInstance
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir
import  random
from enum import  Enum
from  drawEnv import drawPic,drawSTCPic,drawSTCGraph
from math import floor
from itertools import chain

class LawnMainDir(Enum):
    right = 1
    top  = 2
    left = 3
    bottom = 4

class STCEvaluator(object):
    def __init__(self,ins):
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
        self._stcGraph = self._s_map._stcGraph
        self._vitualIndSet = self._s_map._vitualIndSet
        self._stcVitualIndSet = self._s_map._stcVitualIndSet
        self._waySTCNodeNum = self._s_map._waySTCNodeNum


        self._parternLst = []
        self._parternLst.append([LawnMainDir.right,LawnMainDir.top])
        self._parternLst.append([LawnMainDir.right,LawnMainDir.bottom])

        self._parternLst.append([LawnMainDir.left,LawnMainDir.top])
        self._parternLst.append([LawnMainDir.left,LawnMainDir.bottom])

        self._parternLst.append([LawnMainDir.top,LawnMainDir.left])
        self._parternLst.append([LawnMainDir.top,LawnMainDir.right])

        self._parternLst.append([LawnMainDir.bottom,LawnMainDir.left])
        self._parternLst.append([LawnMainDir.bottom,LawnMainDir.right])

        self._dirDic = dict()
        self._dirDic[LawnMainDir.right] = [LawnMainDir.top, LawnMainDir.right, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.left] = [LawnMainDir.top, LawnMainDir.left, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.top] = [LawnMainDir.left, LawnMainDir.top, LawnMainDir.right]
        self._dirDic[LawnMainDir.bottom] = [LawnMainDir.left, LawnMainDir.bottom, LawnMainDir.right]

        pass

    def evaluate(self, x):

        self._robSetLst = [set() for x in range(self._robNum)]
        self._robParternLst = [[] for x in range(self._robNum)]
        self._robStartSTCIndLst = []
        self._coverSet = set()

        for robID in range(self._robNum):
            stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            self._robSetLst[robID].add(stc_ind)
            self._robStartSTCIndLst.append(stc_ind)
            self._coverSet.add(stc_ind)

        print(list(chain(self._robSetLst)))
        robID = 0
        for ind in x:
            main_dir,first_dir = self._parternLst[floor(ind[0]/0.125)]
            '''
            感觉没有必要那么大
            '''
            rob_step = floor(ind[1] * self._stcGraph.number_of_nodes() * 0.5)
            self._robParternLst[robID].append((main_dir,first_dir,rob_step))
            if robID == (self._robNum-1):
                robID = 0
            else:
                robID += 1



        self._c_robPosLst = []
        for robID in range(self._robNum):
            main_dir,first_dir,rob_step = self._robParternLst[robID][0]
            # print(rob_partern)
            # main_dir = self._parternLst[rob_partern][0]
            # first_dir = self._parternLst[rob_partern][1]
            pos = self.getFirstPos(robID,main_dir,first_dir)
            self._c_robPosLst.append(pos)
            self._robSetLst[robID].add(pos)

        print('waySTCNodeNum', self._waySTCNodeNum)
        print(self._robParternLst)
        pass

        while True:
            for robID in range(self._robNum):



    def getFirstPos(self,robID,mainDir, firstDir):

        # print(firstDir)
        # print(mainDir)

        c_pos = self._robStartSTCIndLst[robID]
        c_robDir = firstDir
        p_robDir = firstDir
        while True:
            pos = self.getNextLawnPos(c_pos,c_robDir)
            print(pos)
            if pos.row == -1 and pos.col == -1:
                p_robDir = c_robDir
                c_robDir = self.getNextDirPos(mainDir,c_robDir,p_robDir)
                if c_robDir == mainDir:
                    pos = self.getNextLawnPos(c_pos,c_robDir)
                    if pos.row == -1 and pos.col == -1:
                        raise Exception('xx')
                        break
                    else:
                        c_pos = pos
                        if pos not in self._robSetLst:
                            return pos
                        c_robDir = self.getNextDirPos(mainDir, c_robDir, p_robDir)
            else:
                c_pos = pos
                if pos not in self._robSetLst:
                    return pos

            # pass

    def getNextLawnPos(self,stc_ind :STCGridInd, dir :LawnMainDir):
        res = STCGridInd(-1,-1,STCVirtualVertType.NoVir)
        neiLst = self._stcGraph.neighbors(stc_ind)
        if dir == LawnMainDir.right:
            for nei in neiLst:
                if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                    return nei
            return res
        if dir == LawnMainDir.left:
            for nei in neiLst:
                if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                    return nei
            return res

        if dir == LawnMainDir.top:
            for nei in neiLst:
                if stc_ind.col + 1 == nei.col and stc_ind.row == nei.row:
                    return nei
            return res

        if dir == LawnMainDir.bottom:
            for nei in neiLst:
                if stc_ind.col - 1 == nei.col and stc_ind.row == nei.row:
                    return nei
            return res

    def getNextUnCoverLawnPos(self,stc_ind :STCGridInd, dir :LawnMainDir):
        res = STCGridInd(-1,-1,STCVirtualVertType.NoVir)
        neiLst = self._stcGraph.neighbors(stc_ind)
        if dir == LawnMainDir.right:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                        return nei
            return res
        if dir == LawnMainDir.left:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                        return nei
            return res

        if dir == LawnMainDir.top:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.col + 1 == nei.col and stc_ind.row == nei.row:
                        return nei
            return res

        if dir == LawnMainDir.bottom:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.col - 1 == nei.col and stc_ind.row == nei.row:
                        return nei
            return res
    def getNextDirPos(self,main_dir,c_dir,p_dir):
        dirSeq = self._dirDic[main_dir]
        c_ind = dirSeq.index(c_dir)
        if c_ind  ==  1:
            if dirSeq.index(p_dir) == 2:
                return dirSeq[0]
            else:
                return dirSeq[2]
        else:
            return dirSeq[1]

    def inRobSet(self,stc_ind:STCGridInd):

        if stc_ind in self._coverSet:
            return True
        return False

    def allCover(self):
        if len(self._coverSet) == self._waySTCNodeNum:
            return True
        else:
            return False
        # self._coverSet = set()

def stcEvaluator(pop):
    fitness  = 0
    for ind in pop:
        for x in ind:
            fitness += x
    return fitness

if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    # ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r20_c20_p0.9_s1000_Outdoor_Cfg.dat')
    stc_eval = STCEvaluator(ins)

#
    pop = []
    for i in range(30):
        pop.append((random.random(),random.random()))
    print(pop)

    stc_eval.evaluate(pop)

    # allEdgeLstPnt = []
    # _graph = stc_eval._stcGraph
    # for edge in stc_eval._stcGraph.edges():
    #     # print(edge)
    #     sPnt_x = _graph.nodes[edge[0]]['vert']._pos_x
    #     sPnt_y = _graph.nodes[edge[0]]['vert']._pos_y
    #     tPnt_x = _graph.nodes[edge[1]]['vert']._pos_x
    #     tPnt_y = _graph.nodes[edge[1]]['vert']._pos_y
    #     # edgeLst.append((t_pos_x, t_pos_y, s_pos_x, s_pos_y))
    #
    #     allEdgeLstPnt.append((sPnt_x, sPnt_y, tPnt_x, tPnt_y))
    stcGraphLst = []
    for robID in range(stc_eval._robNum):
        # print(stc_eval._robSetLst)
        _robSet  = stc_eval._robSetLst[robID]
        _stcGraph = []
        for stcGridInd in _robSet:
            _stcGraph.append((stcGridInd.row *2 , stcGridInd.col *2))
        stcGraphLst.append(_stcGraph)
    # print(stcGraphLst)
    drawSTCGraph(ins,stcGraphLst= stcGraphLst)
