from MCMPinstance import MCMPInstance
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir
import  random
from enum import  Enum
from  drawEnv import drawPic,drawSTCPic,drawSTCGraph,drawEvalSTCGraph
from math import floor
from itertools import chain
import  networkx as nx
import  numpy as np
import  math
import  time
from MCMPastar import MCMP_Solver,STC_ASTAR


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
        # print('waySTCNodeNum', self._waySTCNodeNum)
        # print(x)
        self.generateSTree(x)
        if len(self._coverSet) == self._waySTCNodeNum:
            self.generateVirtualPath()
            self.generateRealPath()
            makespan = self.calMakespan()
            return True,makespan
        else:
            return False,np.inf
        # return makespan

    def generateSTree(self,x):
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

        # print(self._robPatternLst)

        self._robSetLst = [[] for x in range(self._robNum)]
        self._robStartSTCIndLst = []
        self._coverSet = set()

        for robID in range(self._robNum):
            stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            self._robSetLst[robID].append(stc_ind)
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
        self._robEndLst = [False for x in range(self._robNum)]

        # for robID,x in enumerate(self._robPatternLst):
        #     # print(x[0])
        #     deg_file.write('robID'+str(robID)+'\n' )
        #     for p in x:
        #         deg_file.write(str(p)+'\n')
        # deg_file.flush()

        for robID in range(self._robNum):
            firstOrdPos,patternInd,rob_step = self._robPatternLst[robID][0]
            firstBool,p_pos,pos = self.getNextBranchPos(robID,firstOrdPos)
            # c_pos,pos = self.getFirstPos(robID,main_dir,first_dir)
            # print('first_pos', pos)
            self._c_sortPatternIndLst.append(patternInd)
            self._c_robPosLst.append(pos)
            # self._c_robDirLst.append(first_dir)
            # self._p_robDirLst.append(first_dir)
            # self._c_mainDirLst.append(main_dir)
            self._robStepLst.append(rob_step)
            self._robSetLst[robID].append(pos)
            # self._robStreeLst[robID].add_node(pos)
            self._robStreeLst[robID].add_edge(p_pos,pos)
            self._coverSet.add(pos)



        # raise  Exception('XX')
        # print(self._robPatternLst)
        pass
            # for y in x:
            #     print(y)
        circleTime = 0
        while False in self._robEndLst:
            for robID in range(self._robNum):
                if self._robEndLst[robID]:
                    continue
                pos = self.getUncoverPos(robID)
                if (pos.row == -1 and  pos.col == -1) or self._c_robStepLst[robID] > self._robStepLst[robID]:
                    self._robPatternStepLst[robID] += 1
                    # print('robID ', robID, ' PatternStep ', self._robPatternStepLst[robID])
                    robPatternStep = self._robPatternStepLst[robID]
                    firstOrdPos, patternInd, rob_step = self._robPatternLst[robID][robPatternStep]
                    firstBool, p_pos,f_pos = self.getNextBranchPos(robID,firstOrdPos)
                    if firstBool:
                        self._coverSet.add(f_pos)
                        self._robSetLst[robID].append(f_pos)
                        self._c_robPosLst[robID] = f_pos
                        self._c_sortPatternIndLst[robID] = patternInd
                        self._c_robStepLst[robID] = 0
                        self._robStreeLst[robID].add_edge(p_pos, f_pos)
                    else:
                        self._robEndLst[robID] = True
                        # raise  Exception('xx')
                else:
                    c_pos = self._c_robPosLst[robID]
                    self._robStreeLst[robID].add_edge(c_pos,pos)
                    self._c_robStepLst[robID] += 1
                    self._robSetLst[robID].append(pos)
                    self._c_robPosLst[robID] = pos
                    self._coverSet.add(pos)
                    # print(pos)
            circleTime += 1
            # print('circletime = ',circleTime)
            # if circleTime == 47:
            #     print('47')
            #     pass
            if circleTime > 200:
                break
        # print('._coverSet = ',len(self._coverSet))

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

    def getNextBranchPos(self,robID, var):
        base_angle = var * 2 * math.pi
        robSet = self._robSetLst[robID]
        robNeiLst = []
        # for stc_ind in robSet:
            # print(stc_ind)

        for stc_ind in robSet:
            # print('stc_ind = ', stc_ind)
            neiLst = self._stcGraph.neighbors(stc_ind)
            for neiInd in neiLst:
                if neiInd not in self._coverSet:
                    # print('neiInd = ',neiInd)
                    angle = self.calAngle(robID,neiInd)
                    robNeiLst.append((stc_ind, neiInd, abs(angle -base_angle)))
                    # print(abs(angle -base_angle))
        robNeiNum  = len(robNeiLst)
        if robNeiNum != 0 :
        # print(robNeiSet)
            robNeiSet = sorted(robNeiLst, key = lambda  x: x[2])
            resInd = floor(robNeiNum * var)
            return True,robNeiSet[resInd][0],robNeiSet[resInd][1]
        else:
            return False, None, None


    def calAngle(self,robID, stc_ind):
        rob_stc_ind = self._robStartSTCIndLst[robID]
        base_x = self._stcGraph.nodes[rob_stc_ind]['vert']._pos_x
        base_y = self._stcGraph.nodes[rob_stc_ind]['vert']._pos_y

        ind_x = self._stcGraph.nodes[stc_ind]['vert']._pos_x
        ind_y = self._stcGraph.nodes[stc_ind]['vert']._pos_y
        y = ind_y - base_x
        x = ind_x - base_x

        if ind_y > base_y:
            return math.atan2(y,x)
        else:
            return math.atan2(y,x) + math.pi

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
    def generateRealPath(self):

        # _mcmp_astar = MCMP_Solver(self._ins)
        self._pathPntLst = [[] for x in range(self._robNum)]
        self._pathLst = [[] for x in range(self._robNum)]


        # print(self._leafVirSetLst)
        # raise Exception('xx')
        for robID in range(self._robNum):
            path = self._pathLst[robID]
            pathPnt = self._pathPntLst[robID]
            virPath = self._virPathLst[robID]

            leafVirLst = self._leafVirSetLst[robID]
            roadPath = []
            for vrow,vcol in virPath:
                if self._mat[vrow][vcol] == 0:
                    roadPath.append((vrow,vcol))
            stc_astar = STC_ASTAR(self._ins,roadPath)
            currentPos = virPath[0]
            path.append(currentPos)
            for i in range(1,len(virPath)):
                nextPos = virPath[i]
                if self._mat[nextPos.row][nextPos.col] == 1:
                    #is obstacle
                    continue

                neiLst = self._s_map.obMapNeighbors(currentPos)
                if len(leafVirLst) != 0:
                    removeBool = False
                    for leafInd in leafVirLst:
                        # print(leafInd)
                        if leafInd in neiLst:
                            # print(leafInd)
                            path.append(leafInd)
                            path.append(currentPos)
                            removeBool = True
                            break
                    if removeBool:
                        leafVirLst.remove(leafInd)

                if nextPos in neiLst:
                    currentPos = nextPos
                    path.append(nextPos)
                else:
                    a_path = list(stc_astar.astar(currentPos,nextPos))
                    currentPos = nextPos
                    path.extend(a_path)
        # print(self._pathLst)

    def generateVirtualPath(self):
        self._virPathPntLst = [[] for x in range(self._robNum)]
        self._virPathLst = [[] for x in range(self._robNum)]
        self._leafVirSetLst = [[] for x in range(self._robNum)]
        for robID in range(self._robNum):
            path = self._virPathLst[robID]
            pathPnt = self._virPathPntLst[robID]
            pos = self._robStartSTCIndLst[robID]
            rob_row = self._ins._robRowLst[robID]
            rob_col = self._ins._robColLst[robID]
            baseInd = GridInd(rob_row,rob_col)
            stree = self._robStreeLst[robID]

            # '''
            # 暂时调试方便
            # '''
            #
            removeLst = []
            leafVirLst = self._leafVirSetLst[robID]
            for streeInd in stree:
                if streeInd in self._stcVitualIndSet and stree.degree(streeInd) == 1:
                    removeLst.append(streeInd)
            for streeInd in removeLst:
                stree.remove_node(streeInd)
                leafVirLst.append(self.getVirBaseInd(streeInd))

            robPathSet = set()
            for stcInd in self._robSetLst[robID]:
                if stcInd.virType == STCVirtualVertType.NoVir:
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LBInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RBInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LTInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RTInd)
                else:
                    if stree.degree(stcInd) == 2:
                        raise Exception('xx')
                        robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LBInd)
                        robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RBInd)
                        robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LTInd)
                        robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RTInd)
                    pass
            cenDir = DirType.center

            while True:
                lastInd = baseInd
                lastDir = cenDir
                path.append(baseInd)
                pathPnt.append((baseInd.row + 0.5, baseInd.col + 0.5))
                # baseNeiLst = self._s_map.baseMapNeighbors(baseInd)
                stc_ind = self._s_map.gridInd2STCGridInd(baseInd)
                stcNeiLst = list (stree.neighbors(stc_ind))
                noIntersectLst = self.intersect(baseInd,stc_ind,stcNeiLst)
                chsSameMega = False
                candLst = []
                for ind in noIntersectLst:
                    if ind in path:
                        continue
                    if ind not in robPathSet:
                        continue
                    if self._s_map.inSameSTCMegaBox(baseInd,ind):
                        baseInd = ind
                        chsSameMega = True
                        break
                    else:
                        candLst.append(ind)
                if not chsSameMega:
                    if len(candLst) != 0:
                        # print(cenDir)
                        # print(candLst)
                        # print('baseInd', baseInd)
                        if len(candLst) == 1 :
                            baseInd = candLst[0]
                            # print('baseInd', baseInd)
                        else:
                            for cand in candLst:
                                candDir = getDir(baseInd, cand)
                                if lastDir == candDir:
                                    baseInd = cand
                                    # if lastDir ==
                                    break

                            if lastDir == DirType.center:
                                baseInd = candLst[-1]
                                # print(baseInd)
                    else:
                        break
                        '''
                        end construct vitual path
                        '''
                cenDir = getDir(lastInd,baseInd)
        # print(self._pathLst)
    def intersect(self, baseInd, stc_ind, stcNeiLst):
        noIntersectLst = []
        baseNeiDirLst = self._s_map.getNeighborDir(baseInd)
        stcNeiDirLst = []
        for stcNeiInd in stcNeiLst:
            stcNeiDir = getDir(stc_ind, stcNeiInd)
            stcNeiDirLst.append((stcNeiDir,stcNeiInd))
            if stcNeiInd.virType != STCVirtualVertType.NoVir:
                print(stcNeiInd)
                # raise Exception('xx')
        for baseDir, baseNeiInd in baseNeiDirLst:
            if baseDir == DirType.left:
                intersectionBool = False
                for stcNeiDir,stcNeiInd in stcNeiDirLst:
                    if stcNeiDir ==  DirType.top:
                        if  stc_ind.col *2  + 1 < baseInd.col + 0.5< stcNeiInd.col *2 + 1:
                            if   baseNeiInd.row + 0.5 < stc_ind.row * 2 + 1 < baseInd.row + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.bottom:
                        if   stcNeiInd.col *2 + 1 < baseInd.col + 0.5 <stc_ind.col* 2 + 1:
                            if   baseNeiInd.row + 0.5 < stc_ind.row * 2 + 1 < baseInd.row + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)

            if baseDir == DirType.right:
                intersectionBool = False
                for stcNeiDir, stcNeiInd in stcNeiDirLst:
                    if stcNeiDir == DirType.top:
                        if stc_ind.col * 2 + 1 < baseInd.col + 0.5 < stcNeiInd.col * 2 + 1:
                            if  baseInd.row + 0.5 < stc_ind.row * 2 + 1 < baseNeiInd.row + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.bottom:
                        if stcNeiInd.col * 2 + 1 < baseInd.col + 0.5 < stc_ind.col * 2 + 1:
                            if  baseInd.row + 0.5 < stc_ind.row * 2 + 1 < baseNeiInd.row + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)

            if baseDir == DirType.top:
                intersectionBool = False
                for stcNeiDir, stcNeiInd in stcNeiDirLst:
                    if stcNeiDir == DirType.left:
                        if stcNeiInd.row*2 + 1 < baseInd.row + 0.5 < stc_ind.row * 2+ 1:
                            if baseInd.col + 0.5 < stc_ind.col*2 + 1 < baseNeiInd.col + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.right:
                        if  stc_ind.row * 2 + 1 < baseInd.row + 0.5 <  stcNeiInd.row * 2 + 1:
                            if baseInd.col + 0.5 < stc_ind.col * 2 + 1 < baseNeiInd.col + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)

            if baseDir == DirType.bottom:
                intersectionBool = False
                for stcNeiDir, stcNeiInd in stcNeiDirLst:
                    if stcNeiDir == DirType.left:
                        if stcNeiInd.row * 2 + 1 < baseInd.row + 0.5 < stc_ind.row * 2 + 1:
                            if  baseNeiInd.col  + 0.5 < stc_ind.col * 2 + 1 < baseInd.col + 0.5:
                                intersectionBool = True
                    if stcNeiDir == DirType.right:
                        if stc_ind.row * 2 + 1 < baseInd.row + 0.5< stcNeiInd.row * 2 + 1:
                            if  baseNeiInd.col + 0.5< stc_ind.col * 2 + 1 <  baseInd.col + 0.5:
                                intersectionBool = True
                if not intersectionBool:
                    noIntersectLst.append(baseNeiInd)
        return noIntersectLst
    def getVirBaseInd(self,stc_ind:STCGridInd):
        if stc_ind.virType == STCVirtualVertType.VLB:
            return self._stcGraph.nodes[stc_ind]['vert']._LBInd
        if stc_ind.virType == STCVirtualVertType.VLT:
            return self._stcGraph.nodes[stc_ind]['vert']._LTInd
        if stc_ind.virType == STCVirtualVertType.VRB:
            return self._stcGraph.nodes[stc_ind]['vert']._RBInd
        if stc_ind.virType == STCVirtualVertType.VRT:
            return self._stcGraph.nodes[stc_ind]['vert']._RTInd


    def calMakespan(self):
        maxPath = max(self._pathLst, key =  lambda x: len(x))
        return len(maxPath)

# def stcEvaluator(pop):
#     fitness  = 0
#     for ind in pop:
#         for x in ind:
#             fitness += x
#     return fitness

if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\pycode\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    # ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r20_c20_p0.9_s1000_Outdoor_Cfg.dat')
    stc_eval = STCEvaluator(ins)

    # random.seed(6)
    # pop = []
    # for i in range(100):
    #     pop.append((random.random(),random.random(),random.random()))

    for seed in range(100):
        random.seed(seed)
        print(seed)
        pop = []
        for i in range(100):
            pop.append((random.random(),random.random(),random.random()))
    # print(pop)
        e_start = time .clock()
        makespan = stc_eval.evaluate(pop)
        print('makespan = ', makespan)
        e_end = time .clock()
        print('evaluate time = ', e_end - e_start)
    # try:
    #     e_start = time .clock()
    #     stc_eval.evaluate(pop)
    #     e_end = time .clock()
    #     print('evaluate time = ', e_end - e_start)
    # except Exception as e:
    #     print(e)
    #     pass

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
    # drawSTCPic(ins,allEdgeLstPnt)
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
    # drawEvalSTCGraph(ins,stcGraphLst= stcGraphLst, edgePntLst = allEdgeLstPnt)
    drawEvalSTCGraph(ins,stcGraphLst= stcGraphLst, edgePntLst = allEdgeLstPnt, multiPath= stc_eval._pathLst)
    # drawPic(ins)
