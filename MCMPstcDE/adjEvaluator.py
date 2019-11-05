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
from stcMap.stc2path import STC2Path
import functools


# class LawnMainDir(Enum):
#     right = 1
#     top  = 2
#     left = 3
#     bottom = 4
def getDirVal(dir):
    if dir == DirType.top:
        val = 1
    if dir == DirType.bottom:
        val = 2
    if dir == DirType.left:
        val = 3
    if dir == DirType.right:
        val = 4
    return val

def getVirtualTypeVal(vir_type):
    if vir_type == STCVirtualVertType.NoVir:
        val = 0
    if vir_type == STCVirtualVertType.VLT:
        val = 1
    if vir_type == STCVirtualVertType.VLB:
        val = 2
    if vir_type == STCVirtualVertType.VRT:
        val = 3
    if vir_type == STCVirtualVertType.VRB:
        val = 4
    return val

def cmpNeiDir(neidir1, neidir2):
    # print(neidir1)
    # print(neidir2)
    val1 = getDirVal(neidir1[1])
    val2 = getDirVal(neidir2[1])
    # print(val1)
    # print(val2)
    if val1 < val2:
        return -1
    elif val1 > val2:
        return 1
    else:
        return 0
    # return False

def cmpSTCVertInd(stc_ind1 :STCGridInd , stc_ind2 : STCGridInd):
    if stc_ind1.row < stc_ind2.row:
        return  -1
    elif stc_ind1.row > stc_ind2.row:
        return 1
    else:
        if stc_ind1.col < stc_ind2.col:
            return -1
        elif stc_ind1.col > stc_ind2.col:
            return 1
        else:
            val1 = getVirtualTypeVal(stc_ind1.virType)
            val2 = getVirtualTypeVal(stc_ind2.virType)
            if val1 < val2:
                return -1
            elif val1 > val2:
                return 1
            else:
                return 0


class Adj_Evaluator(object):
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

        self.stcConvertPath = STC2Path(self._ins, self._s_map)

        self._adjGraph = nx.Graph()
        for node in self._stcGraph.nodes():
            if self._stcGraph.nodes[node]['vert']._type == STCVertType.obstacle:
                continue
            self._adjGraph.add_node(node)

        self._indDic = dict()
        for ind, nodeInd in enumerate(self._adjGraph.nodes):
            self._indDic[ind] = nodeInd
        pass
        self._robStcIndLst = []
        for robID in range(self._robNum):
            stc_ind = self._s_map.gridInd2STCGridInd(GridInd(row=self._robPosLst[robID][0],
                                                      col = self._robPosLst[robID][1]))
            self._robStcIndLst.append(stc_ind)

        print(self._robStcIndLst)
    def evaluate(self,x):
        recordTimeBool = True
        if recordTimeBool:
            t_start = time.clock()
        self.generateSTree(x)
        if recordTimeBool:
            t_end = time.clock()
            print(' _runTime ', t_end - t_start)
        for x in list(nx.connected_components(self._adjGraph)):
            print(x)
        # print()
    def generateSTree(self,x):

        self._adjGraph = nx.Graph()
        for node in self._stcGraph.nodes():
            if self._stcGraph.nodes[node]['vert']._type == STCVertType.obstacle:
                continue
            self._adjGraph.add_node(node)
        print(len(list(nx.connected_components(self._adjGraph))))


        findTimes = 0
        for x_ind, x_val in x:
            # if findTimes > 200:
            #     break
            findTimes += 1
            print('findTimes = ', findTimes)
            if findTimes == 300:
                # break
                print('xxxx')

            # nodeInd = self._indDic[x_ind]

            nodeInd = self._indDic[x_ind]
            neiLst = self._stcGraph.neighbors(nodeInd)
            havePathBool, havePathrobID = self.havePath2Rob(nodeInd)

            dirNeiLst = []
            for nei in neiLst:
                # print(nei)
                nei_dir = getDir(nodeInd, nei)
                if self._adjGraph.has_edge(nodeInd,nei):
                    continue
                if nx.has_path(self._adjGraph, nodeInd, nei):
                    continue
                if havePathBool:
                    if self.havePath2ExpRob(havePathrobID,nei):
                        continue
                dirNeiLst.append((nei, nei_dir))
            # print('b', dirNeiLst)
            neiNum = len(dirNeiLst)
            if neiNum != 0:
                dirNeiLst = sorted(dirNeiLst, key=functools.cmp_to_key(cmpNeiDir))
                chsInd = floor(neiNum * x_val)
                self._adjGraph.add_edge(nodeInd, dirNeiLst[chsInd][0])
                # print('chsInd ', dirNeiLst[chsInd][0])
            else:
                # raise  Exception('xx')
                g_comp = list(nx.connected_components(self._adjGraph))
                for g_comp_ind, component in enumerate(g_comp):
                    if nodeInd in component:
                        break
                # if component
                # component method is not stable
                # print(list(component))
                component = sorted(component, key = functools.cmp_to_key(cmpSTCVertInd))
                robNeiLst = []
                robCompBool = False
                if self.robComp(component):
                    robCompBool = True
                for stc_ind in component:
                    c_neiLst = self._stcGraph.neighbors(stc_ind)
                    for neiInd in c_neiLst:
                        if neiInd in component:
                            continue
                        # if nx.has_path(neiInd,)
                        if robCompBool:
                            havePathBool, robID = self.havePath2Rob(neiInd)
                            if havePathBool:
                                # print('xxxx')
                                continue
                        dis = self.calNeiDis(nodeInd,neiInd)
                        robNeiLst.append((stc_ind, neiInd, dis))

                robNeiNum = len(robNeiLst)
                if robNeiNum != 0:
                    # print('b ', robNeiLst)
                    robNeiSet = sorted(robNeiLst, key=lambda x: x[2])
                    # print('a ', robNeiSet)
                    resInd = floor(robNeiNum*x_val)
                    self._adjGraph.add_edge(robNeiSet[resInd][0], robNeiSet[resInd][1])
                    print('sInd  =', robNeiSet[resInd][0], ' tInd = ', robNeiSet[resInd][1])
                    # exit()
                    # break

                len_comp = len(list(nx.connected_components(self._adjGraph)))
                if len_comp == 4:
                    break
                print('len_comp = ', len(list(nx.connected_components(self._adjGraph))))

            '''
            debug
            '''
            compIndLst = []
            componentLst = list(nx.connected_components(self._adjGraph))
            for robSTCInd in self._robStcIndLst:
                for compInd, component in enumerate(componentLst):
                    if robSTCInd in component:
                        # print(compInd)
                        if compInd in compIndLst:
                            raise Exception('xxxx')
                        compIndLst.append(compInd)
                        break
                # nx.connected_components()
                # if
        self.adjGraph2STree()
    def robComp(self,component):
        for robsctInd in self._robStcIndLst:
            if robsctInd in component:
                return True
        return False

    def havePath2Rob(self,stcInd):
        for robID in range(self._robNum):
            rob_stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            if nx.has_path(self._adjGraph,rob_stc_ind, stcInd):
                return True,robID
        return False,np.inf


    def havePath2ExpRob(self,inRobID,stcInd):
        for robID in range(self._robNum):
            if robID == inRobID:
                continue
            rob_stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            if nx.has_path(self._adjGraph,rob_stc_ind, stcInd):
                return True,robID
        return False,np.inf

    def calNeiDis(self,cen_stc_ind, nei_stc_ind):
        vec_base = np.array([self._stcGraph.nodes[cen_stc_ind]['vert']._pos_x, self._stcGraph.nodes[cen_stc_ind]['vert']._pos_y])
        vec = np.array([self._stcGraph.nodes[nei_stc_ind]['vert']._pos_x, self._stcGraph.nodes[nei_stc_ind]['vert']._pos_y])
        dis = np.linalg.norm(vec_base -vec)
        return  dis
    def adjGraph2STree(self):
        self._robStreeLst = [nx.Graph() for x in range(self._robNum)]
        for robID in range(self._robNum):
            robsctInd = self._robStcIndLst[robID]
            #
            lst = []
            lst.append(robsctInd)
            while len(lst) != 0:
                stc_ind = lst.pop()
                neiLst = self._adjGraph.neighbors(stc_ind)
                for nei in neiLst:
                    if (nei,stc_ind) in self._robStreeLst[robID].edges():
                        continue
                    # if nei in self._robStreeLst[robID]:
                    self._robStreeLst[robID].add_edge(nei,stc_ind)
                    lst.append(nei)
            # if robsctInd in component:
    def __str__(self):
        return str(self._ins) + '   adj_evaluator'

if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r4_r50_c50_p0.8_s1000_Outdoor_Cfg.dat')

    adj_eval = Adj_Evaluator(ins)
    rdSeed = 1
    random.seed(rdSeed)
    num_nodes = adj_eval._adjGraph.number_of_nodes()
    np.random.seed(rdSeed)
    # rdPerm = np.random.permutation(num_nodes)
    # print('num_nodes = ', num_nodes)
    # print(rdPerm)
    # exit(rdPerm)
    # ind = []
    # for i in rdPerm:
    #     ind.append((i, np.random.random()))
    ind = []
    for i in range(num_nodes):
        ind.append((np.random.randint(num_nodes),np.random.random()))
    print('num_nodes = ', num_nodes)
    print('ind = ', ind)

    # exit()
    # for i in range(num_nodes):
    #     ind.append(random.random())
    # print(ind)
    adj_eval.evaluate(ind)
    # exit()

    allEdgeLstPnt = []
    edgeLst = []
    for tInd,sInd in adj_eval._adjGraph.edges():
        t_pos_x = adj_eval._stcGraph.nodes[tInd]['vert']._pos_x
        t_pos_y = adj_eval._stcGraph.nodes[tInd]['vert']._pos_y
        s_pos_x = adj_eval._stcGraph.nodes[sInd]['vert']._pos_x
        s_pos_y = adj_eval._stcGraph.nodes[sInd]['vert']._pos_y
        edgeLst.append((t_pos_x, t_pos_y, s_pos_x, s_pos_y))
    allEdgeLstPnt.append(edgeLst)
    drawEvalSTCGraph(ins, edgePntLst=allEdgeLstPnt)

    stcGraphLst = []
    allEdgeLstPnt = []
    for robID in range(adj_eval._robNum):
        # print(stc_eval._robSetLst)
        _robSet = adj_eval._robStreeLst[robID]
        _stcGraph = []
        for stcGridInd in _robSet:
            _stcGraph.append((stcGridInd.row * 2, stcGridInd.col * 2))
        stcGraphLst.append(_stcGraph)

        stree = adj_eval._robStreeLst[robID]
        edgeLst = []
        for edge in stree.edges():
            # print(edge)
            t_pos_x = adj_eval._stcGraph.nodes[edge[0]]['vert']._pos_x
            t_pos_y = adj_eval._stcGraph.nodes[edge[0]]['vert']._pos_y

            s_pos_x = adj_eval._stcGraph.nodes[edge[1]]['vert']._pos_x
            s_pos_y = adj_eval._stcGraph.nodes[edge[1]]['vert']._pos_y

            edgeLst.append((t_pos_x, t_pos_y, s_pos_x, s_pos_y))
        allEdgeLstPnt.append(edgeLst)
    # print(stcGraphLst)
    # drawEvalSTCGraph(ins,stcGraphLst= stcGraphLst, edgePntLst = allEdgeLstPnt)
    drawEvalSTCGraph(ins, stcGraphLst=stcGraphLst, edgePntLst=allEdgeLstPnt)

    exit()
    edgeLst = []
    _graph = adj_eval._stcGraph
    for edge in adj_eval._stcGraph.edges():
        # print(edge)
        sPnt_x = _graph.nodes[edge[0]]['vert']._pos_x
        sPnt_y = _graph.nodes[edge[0]]['vert']._pos_y
        tPnt_x = _graph.nodes[edge[1]]['vert']._pos_x
        tPnt_y = _graph.nodes[edge[1]]['vert']._pos_y
        edgeLst.append((sPnt_x, sPnt_y, tPnt_x, tPnt_y))
    #     # raise Exception('xx')
    drawSTCPic(ins, edgePntLst = edgeLst)
    print(adj_eval)
