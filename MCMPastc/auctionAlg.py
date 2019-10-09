import stcMap.stcMap
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType
from MCMPinstance import  MCMPInstance
import numpy as np
import networkx as nx
import  drawEnv
from drawEnv import  drawSTCGraph


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


            print(auctionInd)

            winnerRobID, sInd, minCost = self.maxBiddingRob(auctionInd)


            if self._robSleepLst[auctioneerRobID] == True:
                continue
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
                    self.loserEarseVert(self._haveAuctionInd[auctionInd], auctionInd)

                    robWinnerSet = self._robSetLst[winnerRobID]
                    robWinnerSet.add(auctionInd)
                    sTree = self._robStreeLst[winnerRobID]
                    sTree.add_edge(auctionInd, sInd)
                    self._haveAuctionInd[auctionInd] = winnerRobID
                    self._robEstCostLst[winnerRobID] = minCost
                    self.updateNeiGraphRobID(winnerRobID)
            print('circleTime = ', circleTime)
            print('robSetLst = ', self._robSetLst)
            if circleTime >= 200:
                break
            if False not in self._robSleepLst:
                break

    def selectAuctioneer(self):
        minEstCost = min(self._robEstCostLst)
        minRobID = self._robEstCostLst.index(minEstCost)
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
                print('fit = ',fit)
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
        self.auction()
        pass

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

    def __str__(self):
        return 'mcmp_astc row = ' + str(self._row) + ' col = ' + str(self._col)


if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\pycode\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    astc = AuctionAlgSTC(ins)
    astc.plan()

    stcGraphLst = []
    stcNeiGraphLst = []
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
    # print(stcGraphLst)
    drawSTCGraph(ins,stcGraphLst,stcNeiGraphLst)
    print('xx')