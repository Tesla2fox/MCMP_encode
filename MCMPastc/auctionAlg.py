import stcMap.stcMap
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType
from MCMPinstance import  MCMPInstance
import numpy as np
import networkx as nx


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
            self._noBidSet.add(ind)
            self._robStreeLst[robID].add_node(ind)
            self.updateNeiGraphRobID(robID)
            self._haveAuctionInd[ind] = robID

        while True:
            auctioneerID = self.selectAuctioneer()
            auctionInd,allAssign = self.calAcutionVertID(auctioneerID)
            winnerRobID, sInd, cost = self.maxBiddingRob(auctionInd)
            if self._robSleepLst[auctioneerID] == True:
                continue
            if allAssign == False:

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
                robOpNeiLst.append((opRobID,ind,self._robEstCostLst[opRobID]))
            else:
                fit = self.calUnOpPriority(auctioneerID,ind)
                if self.cmpUnOp(fit,minFit):
                    minFit = fit
                    allAssign = False
                    auctionInd = ind

        if allAssign == False:
            return auctionInd,allAssign
        '''
        end the rule 1 
        '''
        raise  Exception('end the rule 1')

    def maxBiddingRob(self,autionInd:STCGridInd):
        maxBidding = -1
        for robID in range(self._robNum):
            # if autionInd inself._robNeiSetLst[robID]
            costLst = []
            biddingLst = []
            sInd,cost = self.calCost(robID, autionInd)
            costLst.append((sInd,cost))
            bidding = 1/cost
            biddingLst.append(bidding)

        maxElement = max(costLst)
        winnerRobID = costLst.index(maxElement)
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
            if fit1[1] > fit2[2]:
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

            minInd, minCost = min(candValLst,lambda x: x[1])
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
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.doubleDiff:
            cost = 2
        if self._stcGraph.nodes[sInd]['vert']._type == STCVertType.wayVert:
            cost = 4

        if self._stcGraph.nodes[tInd]['vert']._type == STCVertType.doubleSame and degree(xx):
            cost += 2
        if self._stcGraph.nodes[tInd]['vert']._type == STCVertType.doubleDiff and degree(xx):
            cost += 2
        return cost
        # self.estAddCost()
    def plan(self):
        print(self._s_map)
        pass

    def calUnOpPriority(self,robID :int ,stcGridInd: STCGridInd):

        # robSet = self._robSetLst[robID]
        fitNess =  0
        for i in range(self._robNum):
            robSet = self._robSetLst[robID]
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

    def distanceSTCInd(self,sInd,tInd):
        vec1 = np.array([self._stcGraph.nodes[sInd]['vert']._pos_x,self._stcGraph.nodes[sInd]['vert']._pos_y])
        vec2 = np.array([self._stcGraph.nodes[tInd]['vert']._pos_x,self._stcGraph.nodes[tInd]['vert']._pos_y])
        '''
        欧式距离
        '''
        return np.linalg.norm(vec1 - vec2, ord = 2)



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

    def formSpanningTree(self):
        pass

    def __str__(self):
        return 'mcmp_astc row = ' + str(self._row) + ' col = ' + str(self._col)


if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\pycode\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    astc = AuctionAlgSTC(ins)


    print('xx')