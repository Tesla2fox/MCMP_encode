from MCMPinstance import MCMPInstance
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir,VirConnectError
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


class STC2Path(object):
    def __init__(self,ins,s_map):
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

        self._s_map = s_map
        self._stcGraph = s_map._stcGraph
        self._vitualIndSet = self._s_map._vitualIndSet
        self._stcVitualIndSet = self._s_map._stcVitualIndSet
        self._waySTCNodeNum = self._s_map._waySTCNodeNum


    def generatePath(self,robStartSTCIndLst, robStreeLst, robSetLst):
        self._robStartSTCIndLst = robStartSTCIndLst
        # print(self._robStartIndLst)
        self._robStreeLst = robStreeLst
        self._robSetLst = robSetLst
        self.generateVirtualPath()
        self.generateRealPath()
        makespan = self.calMakespan()
        return self.calMakespan()
        pass
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

            self._reDefineNeighborAdd = dict()
            self._reDefineNeighborRemove = dict()

            robPathSet = set()
            for stcInd in self._robSetLst[robID]:
                if stcInd.virType == STCVirtualVertType.NoVir:
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LBInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RBInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LTInd)
                    robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RTInd)
                else:
                    if stree.degree(stcInd) == 2:
                        # raise Exception('xx')
                        self.reDefineNeighbor(stcInd)
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
                try:
                    cenDir = getDir(lastInd,baseInd)
                except VirConnectError as e:
                    cenDir = DirType.virConnect
        # print(self._pathLst)
    def intersect(self, baseInd, stc_ind, stcNeiLst):
        noIntersectLst = []
        baseNeiDirLst = self._s_map.getNeighborDir(baseInd)
        if baseInd in self._reDefineNeighborAdd:
            for baseNei in baseNeiDirLst:
                if baseNei[1] == self._reDefineNeighborRemove[baseInd]:
                    break
            baseNeiDirLst.remove(baseNei)
            baseNeiDirLst.append((DirType.virConnect, self._reDefineNeighborAdd[baseInd]))
            print(baseNeiDirLst)

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

            if baseDir == DirType.virConnect:
                noIntersectLst.append(baseNeiInd)
        return noIntersectLst
    def reDefineNeighbor(self,stcInd: STCGridInd):
        if stcInd.virType ==  STCVirtualVertType.VLB:
            ltind = self._stcGraph.nodes[stcInd]['vert']._LTInd
            rbind = self._stcGraph.nodes[stcInd]['vert']._RBInd
            nltind = GridInd(ltind.row - 1 ,ltind.col)
            nrbind = GridInd(rbind.row, rbind.col - 1)
            self._reDefineNeighborAdd[nltind] = nrbind
            self._reDefineNeighborAdd[nrbind] = nltind
            self._reDefineNeighborRemove[nltind] = ltind
            self._reDefineNeighborRemove[nrbind] = rbind

            # self._reDefineNeighbor[ltind] =

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



