from MCMPinstance import MCMPInstance
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir
import  random
from enum import  Enum
from  drawEnv import drawPic,drawSTCPic,drawSTCGraph
from math import floor
from itertools import chain
import  networkx as nx
import  numpy as np

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


        self._patternLst = []
        self._patternLst.append([LawnMainDir.right, LawnMainDir.top, LawnMainDir.bottom, LawnMainDir.left])
        self._patternLst.append([LawnMainDir.right, LawnMainDir.bottom, LawnMainDir.top, LawnMainDir.left])

        self._patternLst.append([LawnMainDir.left, LawnMainDir.top, LawnMainDir.bottom, LawnMainDir.right])
        self._patternLst.append([LawnMainDir.left, LawnMainDir.bottom, LawnMainDir.top, LawnMainDir.right])

        self._patternLst.append([LawnMainDir.top, LawnMainDir.left, LawnMainDir.right, LawnMainDir.bottom])
        self._patternLst.append([LawnMainDir.top, LawnMainDir.right, LawnMainDir.left, LawnMainDir.bottom])

        self._patternLst.append([LawnMainDir.bottom, LawnMainDir.left, LawnMainDir.right, LawnMainDir.top])
        self._patternLst.append([LawnMainDir.bottom, LawnMainDir.right, LawnMainDir.left, LawnMainDir.top])

        self._dirDic = dict()
        self._dirDic[LawnMainDir.right] = [LawnMainDir.top, LawnMainDir.right, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.left] = [LawnMainDir.top, LawnMainDir.left, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.top] = [LawnMainDir.left, LawnMainDir.top, LawnMainDir.right]
        self._dirDic[LawnMainDir.bottom] = [LawnMainDir.left, LawnMainDir.bottom, LawnMainDir.right]

        self._oppoDirDic = dict()
        self._oppoDirDic[LawnMainDir.right] = LawnMainDir.left
        self._oppoDirDic[LawnMainDir.left] = LawnMainDir.right
        self._oppoDirDic[LawnMainDir.top] = LawnMainDir.bottom
        self._oppoDirDic[LawnMainDir.bottom] = LawnMainDir.top
        pass

    def evaluate(self, x):

        print('waySTCNodeNum', self._waySTCNodeNum)


        self._robPatternLst = [[] for x in range(self._robNum)]
        robID = 0
        for ind in x:
            # main_dir,first_dir = self._patternLst[floor(ind[0]/0.125)]
            firstOrdPos = ind[0]
            patternInd = floor(ind[1]/0.125)
            '''
            感觉没有必要那么大
            '''
            rob_step = floor(ind[2] * self._stcGraph.number_of_nodes() * 0.5)
            self._robPatternLst[robID].append((firstOrdPos,patternInd,rob_step))

            if robID == (self._robNum-1):
                robID = 0
            else:
                robID += 1

        print(self._robPatternLst)

        self._robSetLst = [set() for x in range(self._robNum)]
        self._robStartSTCIndLst = []
        self._coverSet = set()

        for robID in range(self._robNum):
            stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            self._robSetLst[robID].add(stc_ind)
            self._robStartSTCIndLst.append(stc_ind)
            self._coverSet.add(stc_ind)

        self._c_robPosLst = []
        self._c_robDirLst = []
        self._p_robDirLst = []
        self._c_mainDirLst = []
        self._c_robStepLst = [0 for x in range(self._robNum)]
        self._robStepLst  = []
        self._robPatternStepLst = [0 for x in range(self._robNum)]
        self._robStreeLst = [nx.Graph() for x in range(self._robNum)]
        self._c_sortPatternIndLst = []
        # for robID,x in enumerate(self._robPatternLst):
        #     # print(x[0])
        #     deg_file.write('robID'+str(robID)+'\n' )
        #     for p in x:
        #         deg_file.write(str(p)+'\n')
        # deg_file.flush()

        for robID in range(self._robNum):
            firstOrdPos,patternInd,rob_step = self._robPatternLst[robID][0]
            firstBool,pos = self.getFirstPosByDis(robID,firstOrdPos)
            # c_pos,pos = self.getFirstPos(robID,main_dir,first_dir)
            print('first_pos', pos)
            self._c_sortPatternIndLst.append(patternInd)
            self._c_robPosLst.append(pos)
            # self._c_robDirLst.append(first_dir)
            # self._p_robDirLst.append(first_dir)
            # self._c_mainDirLst.append(main_dir)
            self._robStepLst.append(rob_step)
            self._robSetLst[robID].add(pos)
            self._robStreeLst[robID].add_node(pos)
            self._coverSet.add(pos)

        # raise  Exception('XX')
        # print(self._robPatternLst)
        pass
            # for y in x:
            #     print(y)
        circleTime = 0
        while True:
            for robID in range(self._robNum):
                # c_pos = self._c_robPosLst[robID]
                pos = self.getUncoverPos(robID)
                if (pos.row == -1 and  pos.col == -1) or self._c_robStepLst[robID] > self._robStepLst[robID]:
                    self._robPatternStepLst[robID] += 1
                    print('robID ', robID, ' PatternStep ', self._robPatternStepLst[robID])
                    robPatternStep = self._robPatternStepLst[robID]
                    firstOrdPos, patternInd, rob_step = self._robPatternLst[robID][robPatternStep]
                    firstBool, f_pos = self.getFirstPosByDis(robID,firstOrdPos)
                    if firstBool:
                        self._coverSet.add(f_pos)
                        self._robSetLst[robID].add(f_pos)
                        self._c_robPosLst[robID] = f_pos
                        self._c_sortPatternIndLst[robID] = patternInd
                        self._c_robStepLst[robID] = 0
                    else:
                        raise  Exception('xx')
                else:
                    c_pos = self._c_robPosLst[robID]
                    self._robStreeLst[robID].add_edge(c_pos,pos)
                    self._c_robStepLst[robID] += 1
                    self._robSetLst[robID].add(pos)
                    self._c_robPosLst[robID] = pos
                    self._coverSet.add(pos)
            circleTime += 1
            if circleTime > 300:
                break

    def getUncoverPos(self,robID):
        c_pos = self._c_robPosLst[robID]
        # c_robDir = self._c_robDirLst[robID]
        # p_robDir = self._p_robDirLst[robID]
        # c_mainDir = self._c_mainDirLst[robID]
        c_sPatternInd = self._c_sortPatternIndLst[robID]
        sortDirLst = self._patternLst[c_sPatternInd]

        for dir in sortDirLst:
            pos = self.getNextUnCoverLawnPos(c_pos,dir)
            if pos.row == -1 and pos.col == -1:
                continue
            return pos
        return pos

    def getFirstPosByDis(self,robID, var):
        robSet = self._robSetLst[robID]
        robNeiSet = []
        for stc_ind in robSet:
            neiLst = self._stcGraph.neighbors(stc_ind)
            for neiInd in neiLst:
                if neiInd not in self._coverSet:
                    dis = self.calFirstDis(robID,neiInd)
                    robNeiSet.append((neiInd,dis))
        robNeiNum  = len(robNeiSet)
        if robNeiNum != 0 :
        # print(robNeiSet)
            robNeiSet = sorted(robNeiSet, key = lambda  x: x[1])
            resInd = floor(robNeiNum * var)
            return True,robNeiSet[resInd][0]
        else:
            return False, None

    def calFirstDis(self,robID,stc_ind):
        dis = 0
        vec_base = np.array([self._stcGraph.nodes[stc_ind]['vert']._pos_x, self._stcGraph.nodes[stc_ind]['vert']._pos_y])
        for x in range(self._robNum):
            if x == robID:
                continue
            rob_stc_ind = self._robStartSTCIndLst[x]
            vec = np.array([self._stcGraph.nodes[rob_stc_ind]['vert']._pos_x, self._stcGraph.nodes[rob_stc_ind]['vert']._pos_y])
            dis += np.linalg.norm(vec_base - vec)
        return dis

    def getFirstPos(self,robID,mainDir, firstDir):
        c_pos = self._robStartSTCIndLst[robID]
        c_robDir = firstDir
        p_robDir = firstDir

        while True:
            pos = self.getNextLawnPos(c_pos,c_robDir)
            # print(pos)
            if pos.row == -1 and pos.col == -1:
                p_robDir = c_robDir
                c_robDir = self.getNextDirPos(mainDir,c_robDir,p_robDir)
                if c_robDir == mainDir:
                    pos = self.getNextLawnPos(c_pos,c_robDir)
                    if pos.row == -1 and pos.col == -1:
                        return c_pos, pos
                        # raise Exception('xx')
                        # break
                    else:
                        if pos not in self._coverSet:
                            return c_pos,pos
                        c_pos = pos

                        c_robDir = self.getNextDirPos(mainDir, c_robDir, p_robDir)
            else:
                if pos not in self._coverSet:
                    return c_pos, pos
                c_pos = pos


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
    random.seed(1)
    pop = []
    for i in range(45):
        pop.append((random.random(),random.random(),random.random()))
    print(pop)
    try:
        stc_eval.evaluate(pop)
    except Exception as e:
        print(e)
        pass

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
    allEdgeLstPnt = []
    for robID in range(stc_eval._robNum):
        # print(stc_eval._robSetLst)
        _robSet  = stc_eval._robSetLst[robID]
        _stcGraph = []
        for stcGridInd in _robSet:
            _stcGraph.append((stcGridInd.row *2 , stcGridInd.col *2))
        stcGraphLst.append(_stcGraph)

        stree = stc_eval._robStreeLst[robID]
        edgeLst = []
        for edge in stree.edges():
            # print(edge)
            t_pos_x = stc_eval._stcGraph.nodes[edge[0]]['vert']._pos_x
            t_pos_y = stc_eval._stcGraph.nodes[edge[0]]['vert']._pos_y

            s_pos_x = stc_eval._stcGraph.nodes[edge[1]]['vert']._pos_x
            s_pos_y = stc_eval._stcGraph.nodes[edge[1]]['vert']._pos_y

            edgeLst.append((t_pos_x,t_pos_y,s_pos_x,s_pos_y))
        allEdgeLstPnt.append(edgeLst)

    # print(stcGraphLst)
    drawSTCGraph(ins,stcGraphLst= stcGraphLst, edgePntLst = allEdgeLstPnt)
