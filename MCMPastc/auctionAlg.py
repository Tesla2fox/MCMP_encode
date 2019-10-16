import stcMap.stcMap
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir
from MCMPinstance import  MCMPInstance
import numpy as np
import networkx as nx
import  drawEnv
from drawEnv import  drawSTCGraph
import sympy
from math import floor
from MCMPastar import MCMP_Solver
import time


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
        self._stcGraph = self._s_map._stcGraph
        self._vitualIndSet = self._s_map._vitualIndSet
        self._stcVitualIndSet = self._s_map._stcVitualIndSet


    def auction(self):
        '''

        :return:
        '''
        self._robEstCostLst = [0 for x in range(self._robNum)]
        self._robSetLst = [set() for x in range(self._robNum)]
        self._robNeiSetLst = [set() for x in range(self._robNum)]
        self._noBidSet = set()
        self._haveAuctionInd = dict()
        self._robSleepLst = [False for x in range(self._robNum)]
        self._robStreeLst = [nx.DiGraph() for x in range(self._robNum)]
        '''
        参数的初始化
        '''
        for robID in range(self._robNum):
            ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            initCost = self.leafCost(ind)
            self._robEstCostLst[robID] =  initCost
            self._robSetLst[robID].add(ind)
            self._noBidSet.add(ind)
            self._robStreeLst[robID].add_node(ind)
            self.updateNeiGraphRobID(robID)
            self._haveAuctionInd[ind] = robID


        circleTime = 0

        while True:

            circleTime += 1

            auctioneerRobID = self.selectAuctioneer()

            auctionInd,allAssign = self.calAcutionVertID(auctioneerRobID)


            # print(auctionInd)

            # if auctionInd == STCGridInd(-1,-1,STCVirtualVertType.NoVir):
            #     pass
                # raise Exception('xx')

            if False not in self._robSleepLst:
                break

            if self._robSleepLst[auctioneerRobID] == True:
                continue

            winnerRobID, sInd, minCost = self.maxBiddingRob(auctionInd)

            if allAssign == False:
                robWinnerSet = self._robSetLst[winnerRobID]
                robWinnerSet.add(auctionInd)
                sTree = self._robStreeLst[winnerRobID]
                sTree.add_edge(auctionInd,sInd)
                self._haveAuctionInd[auctionInd] = winnerRobID
                self._robEstCostLst[winnerRobID] = minCost
                self.updateNeiGraphRobID(winnerRobID)
                pass
            else:
                if auctioneerRobID == winnerRobID:
                    '''
                    此处存在BUG 需要修复
                    '''
                    raise  Exception('xx')
                    self.loserEarseVert(self._haveAuctionInd[auctionInd], auctionInd)
                    robWinnerSet = self._robSetLst[winnerRobID]
                    robWinnerSet.add(auctionInd)
                    sTree = self._robStreeLst[winnerRobID]
                    sTree.add_edge(auctionInd, sInd)
                    self._haveAuctionInd[auctionInd] = winnerRobID
                    self._robEstCostLst[winnerRobID] = minCost
                    self.updateNeiGraphRobID(winnerRobID)
            # print('circleTime = ', circleTime)
            # print('robSetLst = ', self._robSetLst)
            if circleTime >= 250:
                break

    def selectAuctioneer(self):
        estCostLst = []
        for i in range(self._robNum):
            if self._robSleepLst[i] == False:
                estCostLst.append((i,self._robEstCostLst[i]))
        minEstCost = min(estCostLst, key = lambda  x: x[1])
        minRobID = minEstCost[0]
        return minRobID

    def calAcutionVertID(self,auctioneerID):
        robNeiSet = self._robNeiSetLst[auctioneerID]
        if len(robNeiSet) == 0:
            return False, STCGridInd(-1,-1,STCVirtualVertType.NoVir)

        robOpNeiLst = []
        minFit = (True, -1)
        allAssign = True
        auctionInd = STCGridInd(-1,-1,STCVirtualVertType.NoVir)
        for ind in robNeiSet:
            inotherSet = False
            if ind in self._haveAuctionInd:
                inotherSet = True
                opRobID = self._haveAuctionInd[ind]
                robOpNeiLst.append([opRobID,ind,self._robEstCostLst[opRobID]])
            else:
                fit = self.calUnOpPriority(auctioneerID,ind)
                # print('fit = ',fit)
                if self.cmpUnOp(fit,minFit):
                    minFit = fit
                    allAssign = False
                    auctionInd = ind

        if allAssign == False:
            return auctionInd,allAssign
        '''
        end the rule 1 
        '''
        for i in range(len(robOpNeiLst)):
            minInd,cost = self.calCost(auctioneerID,robOpNeiLst[i][1])
            robOpNeiLst[i][2] = robOpNeiLst[i][2] - cost

        robOpNeiLst = sorted(robOpNeiLst, key = lambda x :x [2], reverse= True)
        maxCandi =  robOpNeiLst[0][2]
        if maxCandi <= 0:
            self._robSleepLst[auctioneerID] = True
            return auctionInd, allAssign
        else:
            self._robSleepLst[auctioneerID] = False

        for opRobID,ind,cost in robOpNeiLst:
        # for i in range(len(robOpNeiLst)):
            if cost <=0:
                self._robSleepLst[auctioneerID] = True
                return auctionInd, allAssign
            else:
                auctionBool, pri = self.calOpPriority(auctioneerID,opRobID,ind)
                if auctionBool:
                    auctionInd = ind
                    return auctionInd,allAssign

        raise  Exception('end the rule 2')

    def maxBiddingRob(self,autionInd:STCGridInd):
        maxBidding = -1
        costLst = []
        biddingLst = []
        for robID in range(self._robNum):
            # if autionInd inself._robNeiSetLst[robID]
            sInd,cost = self.calCost(robID, autionInd)
            costLst.append((sInd,cost))
            bidding = 1/cost
            biddingLst.append(bidding)

        # print(biddingLst)
        maxElement = max(biddingLst)
        winnerRobID = biddingLst.index(maxElement)
        sInd = costLst[winnerRobID][0]
        cost = costLst[winnerRobID][1]
        return winnerRobID,sInd,cost
    def cmpUnOp(self,fit1,fit2):
        '''
        if fit1 have higher priority return true
        else return false
        :param fit1:
        :param fit2:
        :return:
        '''
        if fit1[0] == fit2[0]:
            if fit1[1] > fit2[1]:
                return True
            else:
                return False
        else:
            if fit1[0] == True:
                return False
            else:
                return True
        raise  Exception('cmpUnop bug')

    def calCost(self,robID, auctionInd : STCGridInd):
        cost = np.inf
        minInd = np.inf
        if auctionInd in self._robNeiSetLst[robID]:
            candValLst = []
            candinateLst =  self._stcGraph.neighbors(auctionInd)
            for candinateInd in candinateLst:
                if candinateInd in self._robSetLst[robID]:
                    candValLst.append((candinateInd,self.estAddCost(robID,auctionInd,candinateInd)))

            minInd, minCost = min(candValLst,key = lambda x: x[1])
            minCost += self._robEstCostLst[robID]
            return minInd, minCost
            # xx
        if auctionInd in self._robSetLst[robID]:
            minCost = self._robEstCostLst[robID]
            raise Exception('have not finish')
            # xx
        return minInd,cost

    def estAddCost(self,robID, sInd: STCGridInd, tInd: STCGridInd):
        '''
        esatimate the add cost of rob path
        :param robID:
        :param svd: the svd means the vertex out of the graph
        :param tvd: the tvd means the vertex in the rob graph
        :return: the estimated cost
        '''
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.single:
            cost = 6
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.doubleSame:
            if self._s_map.verticalDouble(sInd,tInd):
                cost  = 2
            else:
                cost  = 4
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.doubleDiff or \
                self._stcGraph.nodes[sInd]['vert']._type == STCVertType.triple:
            cost = 2
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.wayVert:
            cost = 4

        if self._stcGraph.nodes[tInd]['vert']._type == STCVertType.doubleSame and self.degree(robID,tInd) == 0:
            cost += 2
        if self._stcGraph.nodes[tInd]['vert']._type == STCVertType.doubleDiff and self.degree(robID,tInd) == 0:
            cost += 2
        # print(self._stcGraph.nodes[sInd]['vert']._type)
        return cost
        # self.estAddCost()
    def plan(self):
        # print(self._s_map)
        a_start = time.clock()
        self.auction()
        a_end = time.clock()
        print('aution used:',a_end - a_start)
        eval_start = time.clock()
        self.searchVitualPath()
        eval_end = time.clock()
        print('aution used:',eval_end - eval_start)
        self.generateRealPath()
        print('makespan = ', self.calMakespan())
        pass
    def calMakespan(self):
        maxPath = max(self._pathLst, key =  lambda x: len(x))
        return len(maxPath)

    def calUnOpPriority(self,robID :int ,stcGridInd: STCGridInd):
        '''

        :param robID:
        :param stcGridInd:
        :return: betterLeaf  priority is lower than vertexes which are not fitting the leaf node
        '''
        # robSet = self._robSetLst[robID]
        fitNess =  0
        for i in range(self._robNum):
            robSet = self._robSetLst[i]
            if i == robID:
                continue
            for ind in robSet:
                dis = self.distanceSTCInd(ind,stcGridInd)
                fitNess += dis

        if self._stcGraph.nodes[stcGridInd]['vert']._type == STCVertType.doubleSame \
                or self._stcGraph.nodes[stcGridInd]['vert']._type == STCVertType.doubleDiff:
            betterLeaf = True
        else:
            betterLeaf = False
        return betterLeaf,fitNess

    def calOpPriority(self,auctioneerRobID: int, opRobID :int, stcGridInd :STCGridInd):
        robSet = self._robSetLst[opRobID]
        if stcGridInd in self._noBidSet:
            return False,np.inf
        indLst = []
        for ind in robSet:
            if stcGridInd == ind:
                continue
            indLst.append(ind)
        if self._s_map.allConnected(indLst):
            '''
            此处和文章不符合 需要进行修改
            '''
            return True,0
        else:
            return False,np.inf


    def distanceSTCInd(self,sInd,tInd):
        vec1 = np.array([self._stcGraph.nodes[sInd]['vert']._pos_x,self._stcGraph.nodes[sInd]['vert']._pos_y])
        vec2 = np.array([self._stcGraph.nodes[tInd]['vert']._pos_x,self._stcGraph.nodes[tInd]['vert']._pos_y])
        '''
        欧式距离
        '''
        return np.linalg.norm(vec1 - vec2, ord = 2)

    def degree(self,robID,ind):
        stree = self._robStreeLst[robID]
        return stree.in_degree(ind)

    def leafCost(self, ind):
        vertType = self._stcGraph.nodes[ind]['vert']._type
        cost = 1000
        if vertType == STCVertType.single:
            cost = 6
            return cost
        if vertType == STCVertType.doubleSame:
            cost = 2
            return  cost
        if vertType == STCVertType.doubleDiff:
            cost =  2
            return cost
        if vertType == STCVertType.wayVert:
            cost = 4
            return cost
        raise  Exception('cost is error')

    def updateNeiGraphRobID(self,robID):
        robSet = self._robSetLst[robID]
        robNeiSet = self._robNeiSetLst[robID]
        robNeiSet.clear()
        for ind in robSet:
            neiLst = self._stcGraph.neighbors(ind)
            for neiInd in neiLst:
                if neiInd not in robSet:
                    robNeiSet.add(neiInd)
        return False

    def loserEarseVert(self,loserID, auctionInd: STCGridInd):
        robLoserSet = self._robSetLst[loserID]
        robLoserSet.remove(auctionInd)

        raise Exception("xx")
        pass

    def formSpanningTree(self):
        pass

    def searchVitualPath(self):

        self._virPathPntLst = [[] for x in range(self._robNum)]
        self._virPathLst = [[] for x in range(self._robNum)]

        for robID in range(self._robNum):
            print('robID = ',robID)
            path = self._virPathLst[robID]
            pathPnt = self._virPathPntLst[robID]

            stree = nx.Graph()
            stree.add_edges_from(self._robStreeLst[robID].edges())

            '''
            此处需要修改
            '''
            for stcInd in self._stcVitualIndSet:
                if stcInd in stree:
                    stree.remove_node(stcInd)
                    # nei = stree.neighbors(stcInd)
                    # stree.remove_edge((nei,stcInd))

            rob_row = self._ins._robRowLst[robID]
            rob_col = self._ins._robColLst[robID]
            baseInd = GridInd(rob_row,rob_col)
            lastInd = GridInd(rob_row,rob_col)
            cenInd  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            canInd = STCGridInd(cenInd.row,cenInd.col,cenInd.virType)

            robPathSet = set()
            for stcInd in self._robSetLst[robID]:
                robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LBInd)
                robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RBInd)
                robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._LTInd)
                robPathSet.add(self._stcGraph.nodes[stcInd]['vert']._RTInd)

            cenDir = DirType.left
            lastDir = DirType.left


            circleTime = 0
            while True:

                lastInd = baseInd
                cenDir = lastDir
                path.append(baseInd)
                pathPnt.append((baseInd.row + 0.5, baseInd.col + 0.5))
                # print('path = ', path)
                baseNeiLst = self._s_map.baseMapNeighbors(baseInd)
                # print('cenInd',cenInd)
                # print('stcNeiLst',stcNeiLst)
                # exit()
                cenInd = self._s_map.gridInd2STCGridInd(baseInd)
                stcNeiLst = list(stree.neighbors(cenInd))

                baseNeiLstStc = []
                for ind in baseNeiLst:
                    sg1 = sympy.Segment2D(sympy.Point2D(ind[0] + 0.5 ,ind[1] + 0.5),sympy.Point2D(baseInd[0] + 0.5,baseInd[1] + 0.5))
                    # print('sg1 ',sg1)
                    # print('len = ', len(stcNeiLst))
                    intersectionBool = False
                    for stcInd in stcNeiLst:
                        # print(stcInd)
                        pnt1 = sympy.Point2D(self._stcGraph.nodes[cenInd]['vert']._pos_x,self._stcGraph.nodes[cenInd]['vert']._pos_y)
                        pnt2 = sympy.Point2D(self._stcGraph.nodes[stcInd]['vert']._pos_x,self._stcGraph.nodes[stcInd]['vert']._pos_y)
                        sg2 = sympy.Segment2D(pnt1,pnt2)
                        # print('sg2',sg2)
                        # print(sg2.intersection(sg1))
                        if len(sg2.intersection(sg1)) != 0:
                            intersectionBool = True
                            break
                            # baseNeiLstStc.append(ind)
                    if not intersectionBool:
                        baseNeiLstStc.append(ind)
                # is_Intersection(sg1,sg2):
                # print(baseNeiLst)
                # print(baseNeiLstStc)
                chsSameMega = False
                candLst = []
                for ind in baseNeiLstStc:
                    if ind in path:
                        continue
                    if ind not in robPathSet:
                        continue
                    # 需要修改
                    _row = floor(ind.row/2)
                    _col = floor(ind.col/2)
                    if GridInd(_row,_col) in self._vitualIndSet:
                        continue
                    '''
                    需要修改
                    '''
                    if self._s_map.inSameSTCMegaBox(baseInd,ind):
                        baseInd = ind
                        chsSameMega = True
                        break
                    else:
                        candLst.append(ind)
                # print(baseInd)
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
                                if lastDir == getDir(baseInd,cand):
                                    baseInd = cand
                                    break
                        # if cenDir == DirType.center:
                    else:
                        '''
                        end construct vitual path
                        '''
                        break
                # print(lastInd,baseInd)
                lastDir = getDir(lastInd,baseInd)
                circleTime  += 1
                if circleTime > 600:
                    pass
                    break
            # break

                # if baseInd == lastInd

    def generateRealPath(self):
        _mcmp_astar = MCMP_Solver(self._ins)

        self._pathPntLst = [[] for x in range(self._robNum)]
        self._pathLst = [[] for x in range(self._robNum)]

        for robID in range(self._robNum):
            path = self._pathLst[robID]
            pathPnt = self._pathPntLst[robID]
            virPath = self._virPathLst[robID]
            currentPos = virPath[0]
            path.append(currentPos)
            for i in range(1,len(virPath)):
                nextPos = virPath[i]
                if self._mat[nextPos.row][nextPos.col] == 1:
                    #is obstacle
                    continue

                neiLst = self._s_map.obMapNeighbors(currentPos)
                if nextPos in neiLst:
                    currentPos = nextPos
                    path.append(nextPos)
                else:
                    a_path = list(_mcmp_astar.astar(currentPos,nextPos))
                    currentPos = nextPos
                    path.extend(a_path)

    def __str__(self):
        return 'mcmp_astc row = ' + str(self._row) + ' col = ' + str(self._col)


if __name__ == '__main__':
    ins = MCMPInstance()
    # ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r20_c20_p0.9_s1000_Outdoor_Cfg.dat')
    astc = AuctionAlgSTC(ins)
    astc.plan()
    # try:
    #     astc.plan()
    # except:
    #     pass

    stcGraphLst = []
    stcNeiGraphLst = []
    allEdgeLstPnt = []
    for robID in range(astc._robNum):
        _robSet  = astc._robSetLst[robID]
        _stcGraph = []
        for stcGridInd in _robSet:
            _stcGraph.append((stcGridInd.row *2 , stcGridInd.col *2))
        stcGraphLst.append(_stcGraph)
        _robNeiSet = astc._robNeiSetLst[robID]
        _stcNeiGraph =[]
        for stcGridInd in _robNeiSet:
            _stcNeiGraph.append((stcGridInd.row *2 , stcGridInd.col *2))
        stcNeiGraphLst.append(_stcNeiGraph)
        stree = astc._robStreeLst[robID]

        edgeLst = []
        for edge in stree.edges():
            # print(edge)
            t_pos_x = astc._stcGraph.nodes[edge[0]]['vert']._pos_x
            t_pos_y = astc._stcGraph.nodes[edge[0]]['vert']._pos_y

            s_pos_x = astc._stcGraph.nodes[edge[1]]['vert']._pos_x
            s_pos_y = astc._stcGraph.nodes[edge[1]]['vert']._pos_y

            edgeLst.append((t_pos_x,t_pos_y,s_pos_x,s_pos_y))
        allEdgeLstPnt.append(edgeLst)
            # stree.edges
    # print(stcGraphLst)
    # drawSTCGraph(ins,stcGraphLst,stcNeiGraphLst = stcNeiGraphLst)
    drawSTCGraph(ins, stcGraphLst=None, stcNeiGraphLst = None, edgePntLst = allEdgeLstPnt, multiPath = astc._pathLst)

    print('xx')